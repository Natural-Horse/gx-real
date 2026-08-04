#!/usr/bin/env bash
set -euo pipefail

# 实机 VLA 全链路启动：工作站建隧道，robodog 上起腿部 WBC、X5 机械臂和交互 client。
#
# 用法（在工作站执行）：
#   bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml start
#   bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml stop
#
# 注意：涉及真机启动、CAN、ROS 节点，执行前必须先按 docs/实机测试指南.md 完成
# 前检，并确认服务器与机器狗空闲。

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

get_cfg() {
  python3 - "${CONFIG_PATH}" "$1" <<'PYEOF'
import sys
try:
    import yaml
except ImportError:
    yaml = None
path, key = sys.argv[1], sys.argv[2]
if yaml is None:
    sys.exit(1)
with open(path, encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
node = data
for part in key.split("."):
    if not isinstance(node, dict) or part not in node:
        sys.exit(1)
    node = node[part]
if node is None:
    sys.exit(1)
if isinstance(node, bool):
    print("true" if node else "false")
else:
    print(str(node))
PYEOF
}

ENDPOINT="$(get_cfg server.endpoint || true)"
SERVER_SSH="$(get_cfg deploy.server_ssh || true)"
ROBOT_SSH="$(get_cfg deploy.robot_ssh || true)"
FRONT_TOPIC="$(get_cfg real.front_topic || true)"
WRIST_TOPIC="$(get_cfg real.wrist_topic || true)"
INSTRUCTION="$(get_cfg task.instruction || true)"
EPISODE_ID="$(get_cfg task.episode_id || true)"
MODE="$(get_cfg real.mode || true)"
JSONL_OUT="$(get_cfg interactive.jsonl_out || true)"
TMUX_SESSION="$(get_cfg deploy.tmux_session || true)"

TMUX_SESSION="${TMUX_SESSION:-vla_real_all}"
MODE="${MODE:-shadow}"

case "${ACTION}" in
  start)
    # 1) 工作站建立双跳隧道（server -> 工作站 -> robodog）
    if [[ -n "${SERVER_SSH}" && -n "${ROBOT_SSH}" ]]; then
      "${SCRIPT_DIR}/evaluation/manage_remote_vla_tunnel.sh" start "${SERVER_SSH}" "${ROBOT_SSH}"
    else
      echo "[vla] 未配置 server_ssh/robot_ssh，跳过隧道（请确认端口已转发）。"
    fi

    # 2) robodog 上启动腿部 WBC（external_vla 速度源）
    if [[ -n "${ROBOT_SSH}" ]]; then
      ssh -o BatchMode=yes "${ROBOT_SSH}" \
        "tmux kill-session -t ${TMUX_SESSION}_leg 2>/dev/null || true; \
         tmux new-session -d -s ${TMUX_SESSION}_leg 'bash -lc \"cd ~/gx-real && source scripts/setup_env.sh && bash scripts/run_vla_leg12_real.sh 2>&1 | tee logs/vla_leg_${TMUX_SESSION}.log\"'"
      # 3) X5 机械臂（唯一 CAN owner，external_vla 源）
      ssh -o BatchMode=yes "${ROBOT_SSH}" \
        "tmux kill-session -t ${TMUX_SESSION}_arm 2>/dev/null || true; \
         tmux new-session -d -s ${TMUX_SESSION}_arm 'bash -lc \"cd ~/gx-real && source scripts/setup_env.sh && bash scripts/run_vla_arm_real.sh 2>&1 | tee logs/vla_arm_${TMUX_SESSION}.log\"'"
    fi

    # 4) 交互 client（robodog 上，连接本机回环 ws://127.0.0.1:10093）
    if [[ -n "${ROBOT_SSH}" ]]; then
      ssh -o BatchMode=yes "${ROBOT_SSH}" \
        "tmux kill-session -t ${TMUX_SESSION}_client 2>/dev/null || true; \
         tmux new-session -d -s ${TMUX_SESSION}_client 'bash -lc \"cd ~/gx-real && python3 scripts/run_vla_real_interactive.py --config ${CONFIG_PATH} 2>&1 | tee logs/vla_client_${TMUX_SESSION}.log\"'"
    else
      echo "[vla] 未配置 robot_ssh，改为在本机启动 client"
      python3 "${SCRIPT_DIR}/run_vla_real_interactive.py" --config "${CONFIG_PATH}"
    fi
    echo "[vla] 已启动: ${TMUX_SESSION} (leg/arm/client)"
    ;;
  stop)
    if [[ -n "${ROBOT_SSH}" ]]; then
      for name in client arm leg; do
        ssh -o BatchMode=yes "${ROBOT_SSH}" \
          "tmux kill-session -t ${TMUX_SESSION}_${name} 2>/dev/null || true"
      done
    fi
    if [[ -n "${SERVER_SSH}" && -n "${ROBOT_SSH}" ]]; then
      "${SCRIPT_DIR}/evaluation/manage_remote_vla_tunnel.sh" stop "${SERVER_SSH}" "${ROBOT_SSH}"
    fi
    echo "[vla] 已停止: ${TMUX_SESSION}"
    ;;
  check)
    if [[ -n "${SERVER_SSH}" && -n "${ROBOT_SSH}" ]]; then
      "${SCRIPT_DIR}/evaluation/manage_remote_vla_tunnel.sh" check "${SERVER_SSH}" "${ROBOT_SSH}"
    fi
    if [[ -n "${ROBOT_SSH}" ]]; then
      ssh -o BatchMode=yes "${ROBOT_SSH}" \
        "tmux ls | grep ${TMUX_SESSION} || true"
    fi
    ;;
esac
