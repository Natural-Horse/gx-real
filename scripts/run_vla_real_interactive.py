#!/usr/bin/env python3
"""实机 VLA 交互评测入口：一条 YAML 命令启动本地 client。

用法：
  python scripts/run_vla_real_interactive.py \
    --config configs/vla_eval/real_go2_x5.yaml

与仿真入口逻辑一致：
  - NAV 只执行第一个 waypoint（发布 /vla/base_cmd），到位后请求下一次推理；
  - GRASP/PLACE 收到决策后终端阻塞，等待操作者输入数字（1=执行 0=跳过），
    期间不发起新的推理请求；
  - 不启动场景、不重置位置（实机）。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "real-wbc"))

from modules.vla_remote_protocol import (  # noqa: E402
    VLARemoteClient,
    VLARemoteClientConfig,
)


def _load_yaml(path: str | Path) -> dict[str, Any]:
    raw = Path(path).expanduser().resolve()
    if not raw.is_file():
        raise FileNotFoundError(f"config yaml does not exist: {raw}")
    with raw.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"config yaml must be a mapping: {raw}")
    return data


def _apply_overrides(data: dict[str, Any], overrides: list[str]) -> None:
    for raw in overrides:
        if "=" not in raw:
            raise ValueError(f"--override 需要 key=value 格式，got {raw!r}")
        key, value = raw.split("=", 1)
        node = data
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = _coerce(value)


def _coerce(value: str) -> Any:
    lowered = value.strip()
    if lowered in {"null", "None", "~"}:
        return None
    if lowered in {"true", "True"}:
        return True
    if lowered in {"false", "False"}:
        return False
    try:
        return int(lowered)
    except ValueError:
        pass
    try:
        return float(lowered)
    except ValueError:
        pass
    return value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实机 VLA 交互评测 client（yaml 驱动）。"
    )
    parser.add_argument("--config", required=True, help="YAML 评测配置路径")
    parser.add_argument(
        "--override",
        action="append",
        default=[],
        help="点分路径覆盖，如 --override task.instruction='x'（可多次）",
    )
    return parser.parse_args()


def _append_jsonl(path: Path, record: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
        stream.write("\n")


class InteractiveRealClient:
    def __init__(self, cfg: dict[str, Any]) -> None:
        self.server_cfg = cfg.get("server") or {}
        self.task_cfg = cfg.get("task") or {}
        self.real_cfg = cfg.get("real") or {}
        self.interactive_cfg = cfg.get("interactive") or {}

        endpoint = str(
            self.server_cfg.get("endpoint", "ws://127.0.0.1:10093")
        )
        self.client = VLARemoteClient(
            VLARemoteClientConfig(
                endpoint=endpoint,
                connect_timeout_s=float(
                    self.server_cfg.get("connect_timeout_s", 10.0)
                ),
                response_timeout_s=float(
                    self.server_cfg.get("response_timeout_s", 120.0)
                ),
            )
        )
        self.instruction = str(
            self.task_cfg.get("instruction", "")
        )
        self.episode_id = str(
            self.task_cfg.get("episode_id", "real_vla_interactive")
        )
        self.phase = "nav_pick"
        self.frame_index = 0
        self.replan_count = 0
        self.locked_route: str | None = None
        self.locked_subtask: str | None = None
        self.log_path = Path(
            str(self.interactive_cfg.get("jsonl_out", "logs/vla_eval/real_interactive.jsonl"))
        )

    def run(self) -> dict[str, Any]:
        from rclpy import init as rclpy_init, spin, shutdown as rclpy_shutdown
        from robot_state.msg import ArmTargetState, TeleopBaseCommand

        rclpy_init()
        spin_thread = None
        try:
            from rclpy.node import Node
            from sensor_msgs.msg import CompressedImage
            from threading import Thread

            node = Node("vla_real_interactive_client")
            self.ArmTargetState = ArmTargetState
            self.TeleopBaseCommand = TeleopBaseCommand
            self.images: dict[str, tuple[bytes, float]] = {}
            self.current_arm_target = None
            self.node = node

            self.base_pub = node.create_publisher(
                TeleopBaseCommand,
                str(self.real_cfg.get("base_command_topic", "/vla/base_cmd")),
                10,
            )
            self.arm_pub = node.create_publisher(
                ArmTargetState,
                str(self.real_cfg.get("arm_command_topic", "/vla/arm_target")),
                10,
            )
            node.create_subscription(
                CompressedImage,
                str(self.real_cfg.get("front_topic", "/camera/front/image_raw/compressed")),
                lambda msg: self._image_cb("front", msg),
                5,
            )
            node.create_subscription(
                CompressedImage,
                str(self.real_cfg.get("wrist_topic", "/camera/wrist/image_raw/compressed")),
                lambda msg: self._image_cb("wrist", msg),
                5,
            )
            node.create_subscription(
                ArmTargetState,
                str(self.real_cfg.get("arm_target_state_topic", "/arm/target_state")),
                self._arm_target_state_cb,
                10,
            )
            if str(self.real_cfg.get("nav_anchor_mode", "body")) == "odom":
                from nav_msgs.msg import Odometry
                from scipy.spatial.transform import Rotation

                self.Odometry = Odometry
                self.Rotation = Rotation
                self.odom_xyyaw = None
                self.odom_last_time = -1.0
                node.create_subscription(
                    Odometry,
                    str(self.real_cfg.get("odom_topic", "/odom")),
                    self._odom_cb,
                    10,
                )

            # 订阅回调需要 executor 持续 spin；推理循环在主线程同步执行，
            # 因此把 spin 放到后台线程，否则相机/臂状态回调永远不会触发。
            spin_thread = Thread(
                target=spin,
                args=(node,),
                daemon=True,
                name="vla_client_rclpy_spin",
            )
            spin_thread.start()

            health = self.client.health()
            print(f"[vla] server health: {health}", flush=True)
            self.client.reset(self.episode_id)
            summary = self._interactive_loop()
            print(json.dumps(summary, ensure_ascii=False, indent=2))
            return summary
        finally:
            if spin_thread is not None:
                spin_thread.join(timeout=1.0)
            rclpy_shutdown()

    def _image_cb(self, key: str, msg) -> None:
        data = bytes(msg.data)
        if data[:2] == b"\xff\xd8":
            self.images[key] = (data, time.monotonic())

    def _arm_target_state_cb(self, msg) -> None:
        if not bool(getattr(msg, "valid", False)):
            return
        try:
            from modules.vla_safety_adapters import pose7_to_model_arm_target

            target = pose7_to_model_arm_target(
                msg.tcp_target_pose,
                msg.gripper_target,
            )
            self.current_arm_target = target
        except ValueError:
            return

    def _odom_cb(self, msg) -> None:
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = self.Rotation.from_quat(quat).as_euler("xyz")
        _ = roll, pitch
        self.odom_xyyaw = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(yaw),
        )
        self.odom_last_time = time.monotonic()

    def _build_payload(self, *, phase: str, frame_index: int) -> dict[str, Any]:
        import base64

        images = {}
        for key in ("front", "wrist"):
            raw = self.images.get(key)
            if raw is None:
                raise RuntimeError(f"no {key} image")
            images[key] = {
                "encoding": "jpeg_base64",
                "data": base64.b64encode(raw[0]).decode("ascii"),
            }
        payload = {
            "episode_id": self.episode_id,
            "frame_index": frame_index,
            "phase": phase,
            "instruction": self.instruction,
            "images": images,
            "state": {
                "base_velocity_body": [0.0, 0.0, 0.0],
                "arm_tcp_base": list(
                    self.current_arm_target
                    if self.current_arm_target is not None
                    else (0.0,) * 7
                ),
            },
        }
        if self.locked_route is not None:
            payload["locked_route"] = self.locked_route
            payload["locked_subtask"] = self.locked_subtask or ""
        return payload

    def _wait_for_images(self) -> None:
        """等待 front/wrist 两路图像到达（首次给 DDS 发现留时间）。"""

        timeout = float(self.real_cfg.get("image_timeout_s", 0.75))
        # 首次启动额外给 DDS 发现/相机上线留时间，之后由 image_timeout 约束新鲜度。
        deadline = time.monotonic() + max(timeout * 4, 3.0)
        while True:
            missing = [
                key for key in ("front", "wrist") if key not in self.images
            ]
            if not missing:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"no {missing} image within "
                    f"{max(timeout * 4, 3.0):.1f}s"
                )
            time.sleep(0.02)

    def _interactive_loop(self) -> dict[str, Any]:
        records: list[dict[str, Any]] = []
        max_replans = int(self.interactive_cfg.get("nav_max_replans", 64))
        self._wait_for_images()
        while True:
            self.frame_index += 1
            payload = self._build_payload(
                phase=self.phase,
                frame_index=self.frame_index,
            )
            started = time.perf_counter()
            decision = self.client.infer(payload)
            elapsed_ms = round((time.perf_counter() - started) * 1000.0, 3)
            route = str(decision.route)
            subtask = decision.subtask
            if (
                self.locked_route is not None
                and route == self.locked_route
                and (subtask or "") == (self.locked_subtask or "")
            ):
                route = self.locked_route
                subtask = self.locked_subtask
            elif route in {"nav", "grasp", "place"}:
                self.locked_route = route
                self.locked_subtask = str(subtask).strip() if subtask else ""
            record = {
                "frame_index": self.frame_index,
                "phase": self.phase,
                "route": route,
                "subtask": subtask,
                "nav_waypoints": [list(p) for p in decision.nav_waypoints],
                "arm_targets_base": [list(t) for t in decision.arm_targets_base],
                "raw_text": decision.raw_text,
                "timing_ms": elapsed_ms,
                "locked_action": self.locked_route is not None,
            }
            records.append(record)
            self._print_decision(record)
            self._append_jsonl({"event": "decision", **record})

            if route == "nav":
                if self.replan_count >= max_replans:
                    print("[vla] NAV replan limit reached; stopping", flush=True)
                    break
                self.replan_count += 1
                self._execute_nav(decision)
                continue
            if route in {"grasp", "place"}:
                self._gate_arm(decision)
                self.phase = "nav_place" if route == "grasp" else "place"
                continue
            if route in {"done", "recover"}:
                print(
                    f"[vla] terminal route={route} subtask={decision.subtask}",
                    flush=True,
                )
                self._append_jsonl({"event": "terminal", "route": route})
                break
            print(f"[vla] unknown route={route}; stopping", flush=True)
            break

        return {
            "episode_id": self.episode_id,
            "inference_count": self.frame_index,
            "replan_count": self.replan_count,
            "records": records,
        }

    def _execute_nav(self, decision) -> None:
        waypoints = decision.nav_waypoints
        if not waypoints:
            print("[vla] NAV decision has no waypoint; stopping", flush=True)
            return
        waypoint = waypoints[0]
        print(
            f"[vla] NAV waypoint #{self.replan_count}: "
            f"{[round(v, 3) for v in waypoint]}",
            flush=True,
        )
        nav_anchor_mode = str(self.real_cfg.get("nav_anchor_mode", "body"))
        if nav_anchor_mode == "odom":
            from modules.waypoint_adapter import WaypointAdapter, WaypointAdapterConfig

            if self.odom_xyyaw is None or (
                time.monotonic() - self.odom_last_time
                > float(self.real_cfg.get("odom_timeout_s", 0.5))
            ):
                print("[vla] odom unavailable/stale; refusing anchored NAV", flush=True)
                self._publish_base(0.0, 0.0, 0.0)
                return
            adapter = WaypointAdapter(WaypointAdapterConfig())
            adapter.set_waypoints(
                [waypoint],
                current_world_xyyaw=self.odom_xyyaw,
            )
            max_steps = int(
                self.interactive_cfg.get("nav_max_steps_per_waypoint", 500)
            )
            step = 0
            while not adapter.goal_reached and step < max_steps:
                if self.odom_xyyaw is None or (
                    time.monotonic() - self.odom_last_time
                    > float(self.real_cfg.get("odom_timeout_s", 0.5))
                ):
                    break
                command = adapter.compute_command(
                    current_world_xyyaw=self.odom_xyyaw,
                    dt_s=0.1,
                )
                self._publish_base(
                    float(command[0]),
                    float(command[1]),
                    float(command[2]),
                )
                time.sleep(0.1)
                step += 1
            reached = adapter.goal_reached
            self._publish_base(0.0, 0.0, 0.0)
            self._append_jsonl(
                {
                    "event": "nav_exec",
                    "frame_index": self.frame_index,
                    "replan": self.replan_count,
                    "waypoint": list(waypoint),
                    "reached": reached,
                    "steps": step,
                    "anchor": "odom",
                }
            )
            if not reached:
                print("[vla] NAV waypoint not reached (odom timeout)", flush=True)
            return

        # body 模式（无里程计）：按第一个 waypoint 的比例速度持续发布，
        # 直到 watchman/现场操作者确认；到达判定需要现场接入里程计。
        from modules.vla_safety_adapters import WaypointVelocityAdapter

        adapter = WaypointVelocityAdapter()
        hold_steps = int(
            self.interactive_cfg.get("nav_body_hold_steps", 10)
        )
        for _ in range(max(1, hold_steps)):
            proposal = adapter.compute(waypoint, stamp=time.monotonic())
            self._publish_base(
                proposal.command.vx,
                proposal.command.vy,
                proposal.command.yaw_rate,
            )
            time.sleep(0.1)
        self._append_jsonl(
            {
                "event": "nav_exec",
                "frame_index": self.frame_index,
                "replan": self.replan_count,
                "waypoint": list(waypoint),
                "command": proposal.command.as_tuple(),
                "reached": proposal.reached,
                "anchor": "body",
            }
        )

    def _gate_arm(self, decision) -> None:
        from modules.vla_safety_adapters import ArmTargetSafetyGate

        route = str(decision.route)
        targets = decision.arm_targets_base
        if not targets:
            print(f"[vla] {route} decision has no arm target; skipping", flush=True)
            return
        prompt = str(
            self.interactive_cfg.get(
                "arm_confirm_prompt",
                "是否执行 {route}？输入 1=执行 0=跳过: ",
            )
        ).format(route=route, subtask=decision.subtask)
        print(
            f"[vla] {route} subtask={decision.subtask} "
            f"target={[round(v, 3) for v in targets[0]]}",
            flush=True,
        )
        choice = self._ask_number(prompt)
        if choice != 1:
            print(f"[vla] {route} skipped by operator", flush=True)
            self.unlock()
            self._append_jsonl(
                {
                    "event": "arm_gate",
                    "frame_index": self.frame_index,
                    "route": route,
                    "choice": choice,
                }
            )
            return
        self.unlock()
        if str(self.real_cfg.get("mode", "shadow")) != "live":
            print(f"[vla] {route} shadow: target recorded, not published", flush=True)
            self._append_jsonl(
                {
                    "event": "arm_shadow",
                    "frame_index": self.frame_index,
                    "route": route,
                    "target": list(targets[0]),
                }
            )
            return
        gate = ArmTargetSafetyGate()
        result = gate.evaluate(targets[0], current_values=self.current_arm_target)
        if not result.accepted:
            print(f"[vla] {route} rejected: {result.reason}", flush=True)
            return
        self._publish_arm_target(targets[0])

    def _ask_number(self, prompt: str) -> int:
        while True:
            try:
                raw = input(prompt).strip()
                choice = int(raw)
            except (EOFError, KeyboardInterrupt):
                print("[vla] 输入中断，按跳过处理", flush=True)
                return 0
            except ValueError:
                print("[vla] 输入无效，请输入 0 或 1", flush=True)
                continue
            if choice in (0, 1):
                return choice
            print("[vla] 请输入 0 或 1", flush=True)

    def unlock(self) -> None:
        self.locked_route = None
        self.locked_subtask = None
        self._append_jsonl({"event": "unlock"})

    def _publish_base(self, vx: float, vy: float, yaw_rate: float) -> None:
        msg = self.TeleopBaseCommand()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.yaw_rate = float(yaw_rate)
        msg.hold = False
        self.base_pub.publish(msg)

    def _publish_arm_target(self, target) -> None:
        from modules.vla_safety_adapters import model_arm_target_to_pose7

        pose7, gripper_width = model_arm_target_to_pose7(target)
        msg = self.ArmTargetState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.joint_target = [0.0] * 6
        msg.tcp_target_pose = list(pose7)
        msg.gripper_target = float(gripper_width)
        msg.command_frame = "base"
        msg.source = "vla_real_interactive_client"
        msg.valid = True
        self.arm_pub.publish(msg)

    def _print_decision(self, record: dict[str, Any]) -> None:
        line = (
            f"[vla] #{record['frame_index']} phase={record['phase']} "
            f"route={record['route']} subtask={record['subtask']}"
        )
        if record["nav_waypoints"]:
            line += (
                " waypoints="
                + str(
                    [
                        [round(v, 3) for v in p]
                        for p in record["nav_waypoints"]
                    ]
                )
            )
        if record["arm_targets_base"]:
            line += (
                " arm_targets="
                + str(
                    [
                        [round(v, 3) for v in t]
                        for t in record["arm_targets_base"]
                    ]
                )
            )
        if self.interactive_cfg.get("show_raw_text", True) and record["raw_text"]:
            line += f" raw={record['raw_text']!r}"
        if self.interactive_cfg.get("show_timing", True):
            line += f" {record['timing_ms']:.0f}ms"
        print(line, flush=True)

    def _append_jsonl(self, record: dict) -> None:
        _append_jsonl(self.log_path, record)


def main() -> int:
    args = _parse_args()
    cfg = _load_yaml(args.config)
    _apply_overrides(cfg, args.override)
    InteractiveRealClient(cfg).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
