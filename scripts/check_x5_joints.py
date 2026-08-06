#!/usr/bin/env python3
"""实机 X5 关节自检（与臂节点同一套夹爪标定配置）。

SDK 自带 python/examples/calibrate.py 使用默认 RobotConfig（无软件零位偏移），
本机夹爪零位掉电丢失时会因静止位换算 -0.022m 触发启动检查失败。本脚本在构造
控制器前套用 real-wbc/modules/arx5_gripper_calib.py 的标定值，与臂节点一致，
用于上电后确认 6 个关节能读到非零且随运动变化。

用法（robodog，启动臂节点前）：
  source scripts/setup_env.sh
  python3 scripts/check_x5_joints.py --seconds 10

如果 6 个关节长期全零或报 missing feedback，不要启动臂节点，按完整指南排查
（X5 24V / 急停 / CAN H/L / 终端电阻 / 是否有其他进程占用 can0）。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "real-wbc"))


def _no_arm_process() -> bool:
    my_pid = os.getpid()
    out = subprocess.run(
        ["pgrep", "-af", "run_spacemouse_arm.py|run_vla_arm_real.sh"],
        capture_output=True,
        text=True,
        check=False,
    )
    for line in out.stdout.splitlines():
        parts = line.split(None, 1)
        if parts and parts[0] == str(my_pid):
            continue
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="X5 关节自检（同臂节点标定配置）")
    parser.add_argument("--model", default="X5")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    if not _no_arm_process():
        print("ERROR: 已有臂进程占用 can0，请先停止再自检。", file=sys.stderr)
        return 1

    import arx5_interface as arx5

    from modules.arx5_gripper_calib import apply_x5_gripper_calibration

    robot_config = arx5.RobotConfigFactory.get_instance().get_config(args.model)
    apply_x5_gripper_calibration(robot_config)
    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", robot_config.joint_dof
    )
    controller_config.gravity_compensation = False
    controller = arx5.Arx5JointController(
        robot_config,
        controller_config,
        args.interface,
    )
    deadline = time.monotonic() + max(args.seconds, 1.0)
    seen_nonzero = False
    while time.monotonic() < deadline:
        state = controller.get_joint_state()
        pos = state.pos()
        gripper = float(getattr(state, "gripper_pos", float("nan")))
        print(
            ", ".join(f"{v:.3f}" for v in pos)
            + f" | gripper {gripper:.3f} m",
            flush=True,
        )
        if any(abs(float(v)) > 1e-6 for v in pos):
            seen_nonzero = True
        time.sleep(0.5)
    print(
        "X5 关节自检完成："
        + ("关节反馈正常" if seen_nonzero else "警告：6 个关节全程为零，请排查反馈"),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
