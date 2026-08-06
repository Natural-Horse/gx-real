#!/usr/bin/env bash
set -euo pipefail

# 重测 X5 夹爪标定常量（换臂/换电机后使用）。
#
# 说明：本机夹爪闭合零位掉电易失，但节点已用软件零位偏移自动映射
# （modules/arx5_gripper_calib.py 的 X5_GRIPPER_ZERO_OFFSET），因此
# 每次上电启动臂节点前【不再需要】运行本脚本。只有在更换机械臂或夹爪电机后，
# 才需要重新实测并更新 arx5_gripper_calib.py 里的两个常量。
#
# 用法（robodog 上）：
#   bash scripts/calibrate_gripper.sh
#
# 流程：自动绕过 SDK 启动检查 -> calibrate_gripper（提示时手动闭合/打开），
# 打印的 fully-open readout 即新的 X5_GRIPPER_OPEN_READOUT。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 1) 确保没有臂节点占用 can0
if pgrep -f "[r]un_spacemouse_arm|[r]un_vla_arm" >/dev/null; then
  echo "ERROR: 请先停止正在运行的臂节点再标定。" >&2
  exit 1
fi

# 2) can0 必须 UP
if ! ip -details link show can0 2>/dev/null | grep -q "state UP"; then
  echo "WARN: can0 未 UP，尝试 setup_arx_can.sh"
  CAN_DEV="$("${ROOT}/scripts/find_can_dev.sh" 2>/dev/null || true)"
  if [[ -z "${CAN_DEV}" ]]; then
    echo "ERROR: 未找到 USB-CAN 设备，请检查连接后重试。" >&2
    exit 1
  fi
  "${ROOT}/scripts/setup_arx_can.sh" "${CAN_DEV}" can0 8
  ip -details link show can0
fi

# 3) 标定（临时 open_readout 绕过启动检查；当前 rest readout≈-1.265 -> 0.037m）
cd "${ROOT}"
source scripts/setup_env.sh
/usr/bin/python3 - <<'EOF'
import arx5_interface as arx5
rc = arx5.RobotConfigFactory.get_instance().get_config("X5")
rc.gripper_open_readout = -3.0  # 仅用于绕过启动检查，标定后以打印值为准
cc = arx5.ControllerConfigFactory.get_instance().get_config("joint_controller", 6)
jc = arx5.Arx5JointController(rc, cc, "can0")
jc.calibrate_gripper()
EOF

echo ""
echo "======================================================"
echo "实测完成。本机日常启动【不需要】标定：零位偏移已在"
echo "real-wbc/modules/arx5_gripper_calib.py 中配置并自动应用。"
echo "仅当换臂/换电机时，才把打印的 fully-open readout 更新到"
echo "arx5_gripper_calib.py 的 X5_GRIPPER_OPEN_READOUT。"
echo "======================================================"
