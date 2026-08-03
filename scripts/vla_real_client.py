#!/usr/bin/env python3
"""ROS2 client for remote StarVLA shadow or explicitly armed live evaluation."""

from __future__ import annotations

import argparse
import base64
from dataclasses import asdict
from datetime import datetime, timezone
import json
import numpy as np
from pathlib import Path
import sys
from threading import Lock, Thread
import time
from typing import Dict, Optional, Tuple


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "real-wbc"))

from modules.vla_remote_protocol import VLARemoteClient, VLARemoteClientConfig
from modules.base_command_provider import BaseCommand
from modules.vla_safety_adapters import (
    ArmTargetSafetyGate,
    WaypointVelocityAdapter,
    model_arm_target_to_pose7,
    pose7_to_model_arm_target,
)
from modules.waypoint_adapter import WaypointAdapter, WaypointAdapterConfig


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--endpoint", default="ws://127.0.0.1:10093")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--episode-id", default="real_eval_0000")
    parser.add_argument("--front-topic", default="/camera/front/image_raw/compressed")
    parser.add_argument("--wrist-topic", default="/camera/wrist/image_raw/compressed")
    parser.add_argument("--arm-target-state-topic", default="/arm/target_state")
    parser.add_argument("--base-command-topic", default="/vla/base_cmd")
    parser.add_argument("--arm-command-topic", default="/vla/arm_target")
    parser.add_argument("--infer-hz", type=float, default=2.0)
    parser.add_argument(
        "--nav-anchor-mode",
        choices=("body", "odom"),
        default="body",
        help="body: 单点 body-frame 比例速度（无需里程计）；odom: 世界系锚定 waypoint 跟踪，需要 --odom-topic。",
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--odom-timeout-s", type=float, default=0.5)
    parser.add_argument("--image-timeout-s", type=float, default=0.75)
    parser.add_argument("--decision-watchdog-s", type=float, default=0.75)
    parser.add_argument("--response-timeout-s", type=float, default=120.0)
    parser.add_argument("--jsonl-out", default="logs/vla_eval/real_eval.jsonl")
    parser.add_argument("--mode", choices=("shadow", "live"), default="shadow")
    parser.add_argument("--enable-live-output", action="store_true")
    parser.add_argument("--confirm-live-output", default="")
    return parser


def _append_jsonl(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
        stream.write("\n")


class RemoteVLARealNode:
    def __init__(self, args: argparse.Namespace) -> None:
        from rclpy.node import Node
        from robot_state.msg import ArmTargetState, TeleopBaseCommand
        from sensor_msgs.msg import CompressedImage

        self.node = Node("remote_vla_real_client")
        self.ArmTargetState = ArmTargetState
        self.TeleopBaseCommand = TeleopBaseCommand
        self.args = args
        self.live = args.mode == "live"
        if self.live and not (
            args.enable_live_output
            and args.confirm_live_output == "I_UNDERSTAND_LIVE_OUTPUT"
        ):
            raise RuntimeError(
                "live mode requires --enable-live-output and "
                "--confirm-live-output I_UNDERSTAND_LIVE_OUTPUT"
            )
        self.client = VLARemoteClient(
            VLARemoteClientConfig(
                endpoint=args.endpoint,
                response_timeout_s=args.response_timeout_s,
            )
        )
        self.nav_anchor_mode = args.nav_anchor_mode
        self.odom_topic = args.odom_topic
        self.odom_timeout_s = args.odom_timeout_s
        self.nav_adapter = WaypointVelocityAdapter()
        self.anchored_nav_adapter = WaypointAdapter()
        self.odom_xyyaw: Optional[Tuple[float, float, float]] = None
        self.odom_last_time = -1.0
        self.arm_gate = ArmTargetSafetyGate()
        self.lock = Lock()
        self.images: Dict[str, Tuple[bytes, float]] = {}
        self.current_arm_target: Optional[Tuple[float, ...]] = None
        self.inference_running = False
        self.frame_index = 0
        self.phase = "nav_pick"
        self.last_decision_time = -1.0
        self.last_zero_publish_time = -1.0
        self.log_path = Path(args.jsonl_out)

        self.base_pub = self.node.create_publisher(
            TeleopBaseCommand, args.base_command_topic, 10
        )
        self.arm_pub = self.node.create_publisher(
            ArmTargetState, args.arm_command_topic, 10
        )
        self.node.create_subscription(
            CompressedImage,
            args.front_topic,
            lambda msg: self._image_cb("front", msg),
            5,
        )
        self.node.create_subscription(
            CompressedImage,
            args.wrist_topic,
            lambda msg: self._image_cb("wrist", msg),
            5,
        )
        self.node.create_subscription(
            ArmTargetState,
            args.arm_target_state_topic,
            self._arm_target_state_cb,
            10,
        )
        if self.nav_anchor_mode == "odom":
            from nav_msgs.msg import Odometry
            from scipy.spatial.transform import Rotation

            self.Odometry = Odometry
            self.Rotation = Rotation
            self.node.create_subscription(
                Odometry,
                self.odom_topic,
                self._odom_cb,
                10,
            )
        self.node.create_timer(1.0 / args.infer_hz, self._start_inference)
        self.node.create_timer(0.05, self._watchdog)

        health = self.client.health()
        self.client.reset(args.episode_id)
        self.node.get_logger().info(
            f"Remote VLA client ready mode={args.mode} endpoint={args.endpoint} "
            f"health={health}"
        )

    def _image_cb(self, key: str, msg) -> None:
        data = bytes(msg.data)
        if data[:2] != b"\xff\xd8":
            self.node.get_logger().warning(f"Ignoring non-JPEG frame from {key}")
            return
        with self.lock:
            self.images[key] = (data, time.monotonic())

    def _arm_target_state_cb(self, msg) -> None:
        if not bool(msg.valid) or str(msg.command_frame) != "base":
            return
        try:
            target = pose7_to_model_arm_target(
                msg.tcp_target_pose,
                msg.gripper_target,
            )
        except ValueError:
            return
        with self.lock:
            self.current_arm_target = target

    def _odom_cb(self, msg) -> None:
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = self.Rotation.from_quat(quat).as_euler("xyz")
        _ = roll, pitch
        with self.lock:
            self.odom_xyyaw = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(yaw),
            )
            self.odom_last_time = time.monotonic()

    def _start_inference(self) -> None:
        now = time.monotonic()
        with self.lock:
            if self.inference_running:
                return
            if any(key not in self.images for key in ("front", "wrist")):
                return
            if any(
                now - self.images[key][1] > self.args.image_timeout_s
                for key in ("front", "wrist")
            ):
                return
            images = {key: self.images[key][0] for key in ("front", "wrist")}
            arm_target = self.current_arm_target
            frame_index = self.frame_index
            self.frame_index += 1
            self.inference_running = True
        Thread(
            target=self._infer_once,
            args=(images, arm_target, frame_index),
            daemon=True,
        ).start()

    def _infer_once(
        self,
        images: Dict[str, bytes],
        arm_target: Optional[Tuple[float, ...]],
        frame_index: int,
    ) -> None:
        started = time.monotonic()
        try:
            payload = {
                "episode_id": self.args.episode_id,
                "frame_index": frame_index,
                "phase": self.phase,
                "instruction": self.args.instruction,
                "images": {
                    key: {
                        "encoding": "jpeg_base64",
                        "data": base64.b64encode(value).decode("ascii"),
                    }
                    for key, value in images.items()
                },
                "state": {
                    "base_velocity_body": [0.0, 0.0, 0.0],
                    "arm_tcp_base": list(arm_target or (0.0,) * 7),
                },
            }
            decision = self.client.infer(payload)
            self.last_decision_time = time.monotonic()
            outputs = self._route_decision(decision, arm_target)
            self.phase = _next_phase(self.phase, decision.route)
            _append_jsonl(
                self.log_path,
                {
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "mode": self.args.mode,
                    "frame_index": frame_index,
                    "phase": self.phase,
                    "decision": decision.to_dict(),
                    "outputs": outputs,
                    "elapsed_ms": round(
                        (time.monotonic() - started) * 1000.0,
                        3,
                    ),
                },
            )
        except Exception as exc:
            self.node.get_logger().error(f"Remote VLA inference rejected: {exc}")
            self._publish_zero_base()
            _append_jsonl(
                self.log_path,
                {
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "mode": self.args.mode,
                    "frame_index": frame_index,
                    "error": str(exc),
                },
            )
        finally:
            with self.lock:
                self.inference_running = False

    def _route_decision(self, decision, current_arm_target) -> dict:
        outputs: dict[str, object] = {"published": self.live}
        if decision.route == "nav":
            if not decision.nav_waypoints:
                raise ValueError("NAV decision has no waypoint")
            if self.nav_anchor_mode == "odom":
                with self.lock:
                    odom = self.odom_xyyaw
                    odom_time = self.odom_last_time
                if odom is None or time.monotonic() - odom_time > self.odom_timeout_s:
                    raise ValueError("odom unavailable or stale; refusing anchored NAV command")
                waypoints = np.asarray(decision.nav_waypoints, dtype=np.float64)
                self.anchored_nav_adapter.set_waypoints(waypoints, np.asarray(odom))
                command = self.anchored_nav_adapter.compute_command(
                    np.asarray(odom),
                    dt_s=1.0 / self.args.infer_hz,
                )
                proposal = type(
                    "NavProposal",
                    (),
                    {
                        "command": BaseCommand(
                            vx=float(command[0]),
                            vy=float(command[1]),
                            yaw_rate=float(command[2]),
                            stamp=time.monotonic(),
                            source="vla_anchored_waypoint_adapter",
                        )
                    },
                )()
            else:
                proposal = self.nav_adapter.compute(
                    decision.nav_waypoints[0],
                    stamp=time.monotonic(),
                )
            outputs["nav"] = asdict(proposal.command)
            if self.live:
                self._publish_base(
                    proposal.command.vx,
                    proposal.command.vy,
                    proposal.command.yaw_rate,
                )
            return outputs

        self._publish_zero_base()
        if decision.route in {"grasp", "place"}:
            if not decision.arm_targets_base:
                raise ValueError(f"{decision.route} decision has no arm target")
            if current_arm_target is None:
                raise ValueError("current /arm/target_state is unavailable")
            target = decision.arm_targets_base[0]
            gate = self.arm_gate.evaluate(target, current_values=current_arm_target)
            outputs["arm_gate"] = asdict(gate)
            if not gate.accepted:
                raise ValueError(f"arm target rejected: {gate.reason}")
            if self.live:
                self._publish_arm_target(target)
        return outputs

    def _publish_base(self, vx: float, vy: float, yaw_rate: float) -> None:
        msg = self.TeleopBaseCommand()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.yaw_rate = float(yaw_rate)
        msg.hold = False
        self.base_pub.publish(msg)

    def _publish_zero_base(self) -> None:
        if not self.live:
            return
        self._publish_base(0.0, 0.0, 0.0)
        self.last_zero_publish_time = time.monotonic()

    def _publish_arm_target(self, target: Tuple[float, ...]) -> None:
        pose7, gripper_width = model_arm_target_to_pose7(target)
        msg = self.ArmTargetState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.joint_target = [0.0] * 6
        msg.tcp_target_pose = list(pose7)
        msg.gripper_target = float(gripper_width)
        msg.command_frame = "base"
        msg.source = "remote_vla_real_client"
        msg.valid = True
        self.arm_pub.publish(msg)

    def _watchdog(self) -> None:
        if not self.live:
            return
        now = time.monotonic()
        stale = (
            self.last_decision_time < 0.0
            or now - self.last_decision_time > self.args.decision_watchdog_s
        )
        zero_due = (
            self.last_zero_publish_time < 0.0
            or now - self.last_zero_publish_time > 0.1
        )
        if stale and zero_due:
            self._publish_zero_base()

    def shutdown(self) -> None:
        self._publish_zero_base()


def _next_phase(current: str, route: str) -> str:
    if route == "nav":
        return "nav_place" if current == "grasp" else current
    return {
        "grasp": "grasp",
        "place": "place",
        "done": "done",
        "recover": "recover",
    }[route]


def main() -> int:
    args = _parser().parse_args()
    if (
        args.infer_hz <= 0.0
        or args.image_timeout_s <= 0.0
        or args.decision_watchdog_s <= 0.0
    ):
        raise SystemExit("infer rate and watchdog timeouts must be positive")
    import rclpy

    rclpy.init(args=None)
    evaluator = RemoteVLARealNode(args)
    try:
        rclpy.spin(evaluator.node)
    finally:
        evaluator.shutdown()
        evaluator.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
