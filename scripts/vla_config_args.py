#!/usr/bin/env python3
"""从 VLA 统一 YAML 按模块生成节点启动参数（camera/leg/arm）。

用法：
  python3 scripts/vla_config_args.py --config configs/vla_eval/real_go2_x5.yaml --module camera

输出为 shell 可安全分词的参数串（无值参数如开关只在其为 true 时输出）。
client 模块不在此生成参数：client 直接读取 yaml 本身（--config + --override）。
"""

from __future__ import annotations

import argparse
import shlex
from pathlib import Path
from typing import Any

import yaml


def _load(path: str | Path) -> dict[str, Any]:
    raw = Path(path).expanduser().resolve()
    if not raw.is_file():
        raise FileNotFoundError(f"config yaml does not exist: {raw}")
    with raw.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"config yaml must be a mapping: {raw}")
    return data


def _get(data: dict[str, Any], dotted: str) -> Any:
    node: Any = data
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(f"config missing key: {dotted}")
        node = node[part]
    return node


# 每个模块的 (命令行参数名, yaml 点分路径) 列表；list 值展开为多个 token，
# bool 值 True 输出无值开关、False 不输出。
MODULE_ARGS: dict[str, list[tuple[str, str]]] = {
    "camera": [
        ("--front-dev", "camera.front_dev"),
        ("--wrist-dev", "camera.wrist_dev"),
        ("--front-topic", "camera.front_topic"),
        ("--wrist-topic", "camera.wrist_topic"),
        ("--rate", "camera.rate"),
        ("--jpeg-quality", "camera.jpeg_quality"),
    ],
    "leg": [
        ("--device", "leg.device"),
        ("--pose_estimator", "leg.pose_estimator"),
        ("--standup-mode", "leg.standup_mode"),
        ("--base-command-source", "leg.base_command_source"),
        ("--external-base-topic", "leg.external_base_topic"),
        ("--external-base-watchdog-sec", "leg.external_base_watchdog_sec"),
        ("--arm-control-owner", "leg.arm_control_owner"),
        ("--arm-state-topic", "leg.arm_state_topic"),
        ("--arm-target-topic", "leg.arm_target_topic"),
        ("--arm-home-topic", "leg.arm_home_topic"),
        ("--safety-topic", "leg.safety_topic"),
        ("--require-arm-state-for-rl", "leg.require_arm_state_for_rl"),
        ("--gripper-cmd", "leg.gripper_cmd"),
        ("--arm_pose", "leg.arm_pose"),
        ("--arm-reset-pose", "leg.arm_reset_pose"),
    ],
    "arm": [
        ("--model", "arm.model"),
        ("--can-interface", "arm.can_interface"),
        ("--control-source", "arm.control_source"),
        ("--external-target-topic", "arm.external_target_topic"),
        ("--external-target-watchdog-sec", "arm.external_target_watchdog_sec"),
        ("--safety-topic", "arm.safety_topic"),
    ],
}


def _render(arg_name: str, value: Any) -> list[str]:
    if isinstance(value, bool):
        return [arg_name] if value else []
    if isinstance(value, (list, tuple)):
        tokens = [shlex.quote(str(item)) for item in value]
        return [arg_name, *tokens]
    return [arg_name, shlex.quote(str(value))]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--module",
        required=True,
        choices=sorted(MODULE_ARGS),
    )
    args = parser.parse_args()
    data = _load(args.config)
    tokens: list[str] = []
    for arg_name, dotted in MODULE_ARGS[args.module]:
        tokens.extend(_render(arg_name, _get(data, dotted)))
    print(" ".join(tokens))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
