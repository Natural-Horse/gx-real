#!/usr/bin/env python3
"""Query remote StarVLA without importing ROS, SDK, or actuator code."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Optional
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "real-wbc"))

from modules.vla_remote_protocol import (  # noqa: E402
    VLARemoteClient,
    VLARemoteClientConfig,
    jpeg_file_payload,
)
from modules.vla_safety_adapters import (  # noqa: E402
    ArmTargetSafetyGate,
    WaypointVelocityAdapter,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="StarVLA shadow client; never publishes ROS or CAN commands."
    )
    parser.add_argument("--endpoint", default="ws://127.0.0.1:10093")
    parser.add_argument("--health-only", action="store_true")
    parser.add_argument("--front-jpeg")
    parser.add_argument("--wrist-jpeg")
    parser.add_argument("--instruction")
    parser.add_argument("--episode-id", default="shadow")
    parser.add_argument("--frame-index", type=int, default=0)
    parser.add_argument("--phase", default="shadow")
    parser.add_argument("--base-velocity-body", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--arm-tcp-base", nargs=7, type=float)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--response-timeout-s", type=float, default=120.0)
    parser.add_argument("--jsonl-out")
    return parser


def _append_jsonl(path: Optional[str], record: dict) -> None:
    if not path:
        return
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def main() -> int:
    args = build_parser().parse_args()
    client = VLARemoteClient(
        VLARemoteClientConfig(
            endpoint=args.endpoint,
            connect_timeout_s=args.connect_timeout_s,
            response_timeout_s=args.response_timeout_s,
        )
    )
    print("SHADOW MODE: no ROS, CAN, base command, or arm command can be published.")
    health = client.health()
    record = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "mode": "shadow",
        "health": health,
    }
    if args.health_only or args.front_jpeg is None:
        print(json.dumps(record, indent=2, ensure_ascii=False))
        _append_jsonl(args.jsonl_out, record)
        return 0
    if not args.instruction:
        raise SystemExit("--instruction is required when --front-jpeg is provided")

    images = {"front": jpeg_file_payload(args.front_jpeg)}
    if args.wrist_jpeg:
        images["wrist"] = jpeg_file_payload(args.wrist_jpeg)
    arm_tcp = tuple(args.arm_tcp_base or (0.0,) * 7)
    payload = {
        "episode_id": str(args.episode_id),
        "frame_index": int(args.frame_index),
        "phase": str(args.phase),
        "instruction": str(args.instruction),
        "images": images,
        "state": {
            "base_velocity_body": list(args.base_velocity_body),
            "arm_tcp_base": list(arm_tcp),
        },
    }
    client.reset(str(args.episode_id))
    decision = client.infer(payload)
    proposals: dict[str, object] = {}
    if decision.route == "nav":
        proposal = WaypointVelocityAdapter().compute(
            decision.nav_waypoints[0], stamp=time.monotonic()
        )
        proposals["nav_velocity_not_published"] = {
            **asdict(proposal),
            "command": asdict(proposal.command),
        }
    if decision.route in {"grasp", "place"} and decision.arm_targets_base:
        if args.arm_tcp_base is None:
            proposals["arm_target_not_published"] = {
                "accepted": False,
                "reason": "--arm-tcp-base is required for the arm safety gate",
            }
        else:
            proposals["arm_target_not_published"] = asdict(
                ArmTargetSafetyGate().evaluate(
                    decision.arm_targets_base[0], current_values=args.arm_tcp_base
                )
            )
    record.update({"request": {**payload, "images": "omitted"}, "decision": decision.to_dict(), "proposals": proposals})
    print(json.dumps(record, indent=2, ensure_ascii=False))
    _append_jsonl(args.jsonl_out, record)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
