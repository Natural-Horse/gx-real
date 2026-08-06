#!/usr/bin/env bash
set -euo pipefail

# 在 robodog 上用单个 tmux 会话、2x2 四宫格同时运行 4 个 VLA 节点：
#   +------------+------------+
#   | vla_cams   | vla_leg    |
#   +------------+------------+
#   | vla_arm    | vla_client |
#   +------------+------------+
#
# 用法：
#   bash scripts/run_vla_real_quad.sh --config configs/vla_eval/real_go2_x5.yaml start
#   bash scripts/run_vla_real_quad.sh --config configs/vla_eval/real_go2_x5.yaml check
#   bash scripts/run_vla_real_quad.sh --config configs/vla_eval/real_go2_x5.yaml stop
#
# 查看：tmux attach -t go2_vla_quad （Ctrl-B 松开后按方向键切换 pane；
# GRASP/PLACE 时点击 client pane 输入 1/0）。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SESSION="go2_vla_quad"

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

    # 停掉旧的单会话节点，避免 CAN owner / 重复 WBC / 相机占用冲突
    for legacy in vla_cams vla_leg vla_arm vla_client vla_decompress; do
      if tmux has-session -t "${legacy}" 2>/dev/null; then
        echo "[vla] 停止旧会话 ${legacy} ..."
        tmux kill-session -t "${legacy}" 2>/dev/null || true
      fi
    done

    tmux kill-session -t "${SESSION}" 2>/dev/null || true

    # 左上：相机
    tmux new-session -d -s "${SESSION}" -n nodes \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       python3 scripts/publish_real_cameras.py 2>&1 | tee logs/vla_cams.log"
    # 节点退出后 pane 保留显示错误，便于排查
    tmux set-option -t "${SESSION}" remain-on-exit on 2>/dev/null || true
    # 右上：腿部 WBC
    tmux split-window -h -t "${SESSION}:0.0" \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       bash scripts/run_vla_leg12_real.sh 2>&1 | tee logs/vla_leg.log"
    # 左下：X5 机械臂
    tmux split-window -v -t "${SESSION}:0.0" \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       bash scripts/run_vla_arm_real.sh 2>&1 | tee logs/vla_arm.log"
    # 右下：交互 client
    tmux split-window -v -t "${SESSION}:0.1" \
      "cd ${ROOT} && source scripts/setup_env.sh && \
       python3 scripts/run_vla_real_interactive.py --config ${CONFIG_PATH} \
       2>&1 | tee logs/vla_client.log"

    tmux select-layout -t "${SESSION}" tiled
    echo "[vla] 已启动 ${SESSION}（2x2：cams / leg / arm / client）"
    echo "[vla] tmux attach -t ${SESSION} 查看；日志 logs/vla_{cams,leg,arm,client}.log"
    ;;
  stop)
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
    echo "[vla] 已停止 ${SESSION}"
    ;;
  check)
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
      tmux list-panes -t "${SESSION}" -F "#{pane_index} #{pane_current_command} #{pane_title}" 2>/dev/null
    else
      echo "[vla] 没有 ${SESSION} 会话" >&2
      exit 1
    fi
    ;;
esac
