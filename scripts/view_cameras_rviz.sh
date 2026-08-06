#!/usr/bin/env bash
set -euo pipefail

# 一键打开 rviz2 并直接显示 front/wrist 两路相机图像。
# 自动完成：起相机发布器（没有时）-> 起解压节点（没有时）-> 等待图像 ->
# 打开 rviz2（已配置两个 Image 显示）。
#
# 用法（robodog 上）：
#   bash scripts/view_cameras_rviz.sh
#
# 注意：需要图形会话（Jetson 桌面终端，或 ssh -X）；独立于四宫格 tmux，
# 不会动 vla_leg/vla_arm/vla_client。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_RVIZ="${ROOT}/configs/rviz/cameras.rviz"
VLA_CONFIG="${VLA_CONFIG:-${ROOT}/configs/vla_eval/real_go2_x5.yaml}"
CAM_ARGS="$(python3 "${ROOT}/scripts/vla_config_args.py" --config "${VLA_CONFIG}" --module camera 2>/dev/null || true)"

# 1) 相机发布器（未运行时启动 vla_cams）
if ! pgrep -f "[p]ublish_real_cameras.py" >/dev/null; then
  echo "[rviz] 相机发布器未运行，启动 vla_cams ..."
  tmux kill-session -t vla_cams 2>/dev/null || true
  tmux new-session -d -s vla_cams \
    "cd ${ROOT} && source scripts/setup_env.sh && \
     python3 scripts/publish_real_cameras.py ${CAM_ARGS} \
     2>&1 | tee logs/vla_cams.log"
fi

# 2) 解压节点（未运行时启动 vla_decompress）
if ! pgrep -f "[d]ecompress_cameras.py" >/dev/null; then
  echo "[rviz] 解压节点未运行，启动 vla_decompress ..."
  tmux kill-session -t vla_decompress 2>/dev/null || true
  tmux new-session -d -s vla_decompress \
    "cd ${ROOT} && source scripts/setup_env.sh && \
     python3 scripts/decompress_cameras.py 2>&1 | tee logs/vla_decompress.log"
fi

source "${ROOT}/scripts/setup_env.sh"

# 3) 等待两路压缩话题被发布（最多 15s；不依赖 ros2 topic hz，
#    Jetson 上 CycloneDDS 发现慢会导致 hz 超时误判）
echo "[rviz] 等待 front/wrist 话题..."
for i in $(seq 1 15); do
  front_ok=$(timeout 3 ros2 topic list 2>/dev/null | grep -c "/camera/front/image_raw/compressed" || true)
  wrist_ok=$(timeout 3 ros2 topic list 2>/dev/null | grep -c "/camera/wrist/image_raw/compressed" || true)
  if [[ "${front_ok}" -gt 0 && "${wrist_ok}" -gt 0 ]]; then
    break
  fi
  sleep 1
done
if [[ "${front_ok:-0}" -eq 0 || "${wrist_ok:-0}" -eq 0 ]]; then
  echo "WARN: 相机话题还没出现（front=${front_ok:-0} wrist=${wrist_ok:-0}），"
  echo "      仍会打开 rviz2；请检查 logs/vla_cams.log"
fi
# 给 DDS 发现和解压节点留时间，rviz 打开后图像会陆续显示
sleep 5

# 4) 图形显示检查
if [[ -z "${DISPLAY:-}" ]]; then
  if [[ -e /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
    echo "[rviz] DISPLAY 为空，改用 :0"
  else
    echo "ERROR: 找不到 X display。请在 Jetson 桌面终端运行，或 ssh -X 后重试。" >&2
    exit 1
  fi
fi

# 5) 打开 rviz2
echo "[rviz] 启动 rviz2 ..."
exec rviz2 -d "${CONFIG_RVIZ}"
