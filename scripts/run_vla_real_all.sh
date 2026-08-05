#!/usr/bin/env bash
set -euo pipefail

# 实机 VLA 全链路：在 robodog 本机启动腿部 WBC、X5 机械臂和交互 client。
#
# 用法（登录到 robodog 后执行）：
#   cd ~/gx-real
#   bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml start
#   bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml stop
#   bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml check
#
# 注意：本脚本在 robodog 本机运行，只负责三个 ROS/CAN 进程。
# SSH 隧道在工作站单独启动（见 docs/vla_real_client.md 第 4 节），
# 推理服务在 GPU 服务器单独启动（starVLA_sc）。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  echo "用法: $0 --config <real.yaml> <start|stop|check>" >&2
  exit 2
}

CONFIG=""
ACTION=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG="${2:-}"
      shift 2
      ;;
    start|stop|check)
      ACTION="$1"
      shift
      ;;
    *)
      usage
      ;;
  esac
done
[[ -n "${CONFIG}" && -n "${ACTION}" ]] || usage
CONFIG_PATH="$(cd "$(dirname "${CONFIG}")" && pwd)/$(basename "${CONFIG}")"
[[ -f "${CONFIG_PATH}" ]] || {
  echo "config yaml 不存在: ${CONFIG_PATH}" >&2
  exit 2
}

# 上机前检查（robodog 本机）：参考 README 第 5 节
preflight() {
  echo "[vla] 上机前检查..."
  [[ "$(uname -m)" == "aarch64" ]] || {
    echo "FAIL: 必须在 Jetson (aarch64) 上运行" >&2
    exit 1
  }
  source "${ROOT}/scripts/setup_env.sh"
  "${ROOT}/scripts/check_env.sh" || {
    echo "FAIL: check_env.sh 未通过" >&2
    exit 1
  }
  if ! ip -details link show can0 2>/dev/null | grep -q "state UP"; then
    echo "WARN: can0 未 UP，尝试 setup_arx_can.sh"
    "${ROOT}/scripts/setup_arx_can.sh"
    ip -details link show can0
  fi
  echo "[vla] 上机前检查通过"
}

case "${ACTION}" in
  start)
    preflight

    # 真机相机（front/wrist RGB -> JPEG CompressedImage topic）
    tmux kill-session -t vla_cams 2>/dev/null || true
    tmux new-session -d -s vla_cams \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       python3 scripts/publish_real_cameras.py 2>&1 | tee logs/vla_cams.log"

    # 腿部 WBC（external_vla 速度源）
    tmux kill-session -t vla_leg 2>/dev/null || true
    tmux new-session -d -s vla_leg \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       bash scripts/run_vla_leg12_real.sh 2>&1 | tee logs/vla_leg.log"

    # X5 机械臂（唯一 CAN owner，external_vla 源）
    tmux kill-session -t vla_arm 2>/dev/null || true
    tmux new-session -d -s vla_arm \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       bash scripts/run_vla_arm_real.sh 2>&1 | tee logs/vla_arm.log"

    # 交互 client（连接本机回环 ws://127.0.0.1:10093，经工作站隧道到服务器）
    tmux kill-session -t vla_client 2>/dev/null || true
    tmux new-session -d -s vla_client \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       ${ROOT}/scripts/run_vla_real_interactive.py --config ${CONFIG_PATH} \
       2>&1 | tee logs/vla_client.log"

    echo "[vla] 已启动: vla_cams / vla_leg / vla_arm / vla_client"
    echo "[vla] 各 tmux 日志: ~/gx-real/logs/vla_{leg,arm,client}.log"
    echo "[vla] 记得在工作站已执行隧道: manage_remote_vla_tunnel.sh start"
    ;;
  stop)
    for name in cams client arm leg; do
      tmux kill-session -t "vla_${name}" 2>/dev/null || true
    done
    echo "[vla] 已停止 vla_cams / vla_leg / vla_arm / vla_client"
    ;;
  check)
    tmux ls | grep vla_ || echo "[vla] 没有 vla_* 会话"
    ;;
esac
