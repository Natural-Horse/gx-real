#!/usr/bin/env bash
set -euo pipefail

# 上电后标定 X5 夹爪（本机夹爪零位易失，断电/SDK 重启后会失效，
# 必须在上电后、启动臂节点前跑一次本脚本）。
#
# 用法（robodog 上）：
#   bash scripts/calibrate_gripper.sh
#
# 流程：自动绕过 SDK 启动检查 -> calibrate_gripper（提示时手动闭合/打开）
# 完成后保持通电，直接启动臂节点（run_vla_arm_real.sh / run_spacemouse_arm.sh）。

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
  CAN_DEV=$(ls /dev/serial/by-id/usb-Openlight_Labs_CANable2* 2>/dev/null | head -1)
  if [[ -z "${CAN_DEV}" ]]; then
    CAN_DEV=$(ls /dev/serial/by-id/usb-ARX* 2>/dev/null | head -1)
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
echo "标定完成。请保持通电，直接启动臂节点："
echo "  bash scripts/run_vla_arm_real.sh"
echo "若换臂/换 SDK，把打印的 fully-open readout 更新到"
echo "real-wbc/modules/spacemouse_arm_node.py 的 X5_GRIPPER_OPEN_READOUT。"
echo "======================================================"
