"""X5 夹爪标定常量（本机实机实测，2026-08-06）。

本机夹爪的闭合零位在掉电/SDK 重启后会丢失（reset_zero_readout 写入电机的
零位不持久）。因此启动时用软件零位偏移把闭合位映射回 0m，无需每次上电交互式
标定。数值来自同一实测会话（旧 config readout=-5.07839 下读数换算原始 readout）：

  完全打开: pos 0.022  -> raw_open   =  0.022 / 0.088 * -5.07839 = -1.2696
  完全闭合: pos 0.110  -> raw_closed =  0.110 / 0.088 * -5.07839 = -6.3480
  span = raw_open - raw_closed = 5.0784

换算（SDK 已支持 gripper_zero_offset）：
  pos(m) = (angle_actual_rad - gripper_zero_offset) / gripper_open_readout * gripper_width
命令反向：motor_target = gripper_zero_offset + pos/width * gripper_open_readout。

换臂/换电机后需用 SDK calibrate_gripper 重新实测并更新这两个值。
"""

X5_GRIPPER_WIDTH = 0.088
X5_GRIPPER_OPEN_READOUT = 5.0784
X5_GRIPPER_ZERO_OFFSET = -6.348


def apply_x5_gripper_calibration(robot_config) -> None:
    """把本机 X5 夹爪标定值写入 SDK RobotConfig（构造控制器前调用）。"""
    robot_config.gripper_width = X5_GRIPPER_WIDTH
    robot_config.gripper_open_readout = X5_GRIPPER_OPEN_READOUT
    robot_config.gripper_zero_offset = X5_GRIPPER_ZERO_OFFSET
