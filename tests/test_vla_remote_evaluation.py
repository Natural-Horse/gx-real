from pathlib import Path
import json
import sys
from threading import Thread

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "real-wbc"))

from modules.vla_remote_protocol import (  # noqa: E402
    PROTOCOL_VERSION,
    VLAEvaluationDecision,
    VLARemoteClient,
    VLARemoteClientConfig,
    VLARemoteError,
)
from modules.vla_safety_adapters import (  # noqa: E402
    ArmTargetSafetyGate,
    WaypointVelocityAdapter,
    model_arm_target_to_pose7,
    pose7_to_model_arm_target,
)
from modules.spacemouse_arm_node import (  # noqa: E402
    arm_eef_pose6d_to_base_tcp_pose7,
    base_tcp_pose7_to_arm_eef_pose6d,
)


def test_remote_client_only_accepts_loopback_endpoint():
    with pytest.raises(VLARemoteError, match="loopback"):
        VLARemoteClientConfig(endpoint="ws://10.0.0.4:10093")


def test_response_request_id_and_protocol_are_checked():
    raw = json.dumps(
        {
            "protocol_version": PROTOCOL_VERSION,
            "type": "health_result",
            "request_id": "old",
            "ok": True,
            "data": {},
        }
    )
    with pytest.raises(VLARemoteError, match="request_id"):
        VLARemoteClient._validate_response(raw, "new")


def test_nav_decision_and_velocity_proposal_are_bounded():
    decision = VLAEvaluationDecision.from_response(
        {"route": "nav", "nav_waypoints": [[0.8, -0.5, 1.0]]}
    )
    result = WaypointVelocityAdapter().compute(decision.nav_waypoints[0], stamp=1.0)
    assert result.reached is False
    assert abs(result.command.vx) <= 0.20
    assert abs(result.command.vy) <= 0.10
    assert abs(result.command.yaw_rate) <= 0.30


def test_arm_gate_rejects_large_step_and_accepts_small_step():
    gate = ArmTargetSafetyGate()
    current = [0.30, 0.0, 0.20, 0.0, 0.0, 0.0, 0.5]
    rejected = gate.evaluate(
        [0.60, 0.0, 0.20, 0.0, 0.0, 0.0, 0.5], current_values=current
    )
    assert rejected.accepted is False
    assert rejected.reason == "tcp_translation_step_too_large"
    accepted = gate.evaluate(
        [0.34, 0.0, 0.22, 0.0, 0.1, 0.0, 0.6], current_values=current
    )
    assert accepted.accepted is True


def test_grasp_target_requires_valid_gripper_range():
    with pytest.raises(VLARemoteError, match="gripper"):
        VLAEvaluationDecision.from_response(
            {"route": "grasp", "arm_targets_base": [[0.3, 0, 0.2, 0, 0, 0, 1.5]]}
        )


def test_loopback_websocket_health_roundtrip():
    from websockets.sync.server import serve

    def handler(connection):
        request = json.loads(connection.recv())
        connection.send(
            json.dumps(
                {
                    "protocol_version": PROTOCOL_VERSION,
                    "type": "health_result",
                    "request_id": request["request_id"],
                    "ok": True,
                    "data": {"backend": {"name": "mock"}},
                }
            )
        )

    with serve(handler, "127.0.0.1", 0) as server:
        port = server.socket.getsockname()[1]
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            health = VLARemoteClient(
                VLARemoteClientConfig(endpoint=f"ws://127.0.0.1:{port}")
            ).health()
        finally:
            server.shutdown()
            thread.join(timeout=2.0)
    assert health["backend"]["name"] == "mock"


def test_model_arm_target_ros_pose_roundtrip():
    target = (0.31, -0.02, 0.24, 0.1, -0.2, 0.3, 0.75)
    pose7, gripper = model_arm_target_to_pose7(target)
    recovered = pose7_to_model_arm_target(pose7, gripper)
    assert recovered == pytest.approx(target)


def test_base_tcp_and_arx_eef_frame_roundtrip():
    base_tcp = (0.34, 0.02, 0.23, 1.0, 0.0, 0.0, 0.0)
    arm_eef = base_tcp_pose7_to_arm_eef_pose6d(base_tcp)
    recovered = arm_eef_pose6d_to_base_tcp_pose7(arm_eef)
    assert recovered[:3] == pytest.approx(base_tcp[:3])
    assert abs(sum(a * b for a, b in zip(recovered[3:], base_tcp[3:]))) == pytest.approx(1.0)


def test_live_evaluation_sources_keep_explicit_gates_and_watchdogs():
    runner = (ROOT / "scripts" / "vla_real_client.py").read_text(encoding="utf-8")
    wbc = (
        ROOT / "real-wbc" / "modules" / "wbc_node_leg12_arm_passthrough.py"
    ).read_text(encoding="utf-8")
    arm = (ROOT / "real-wbc" / "modules" / "spacemouse_arm_node.py").read_text(
        encoding="utf-8"
    )
    assert "I_UNDERSTAND_LIVE_OUTPUT" in runner
    assert "decision_watchdog_s" in runner
    assert 'base_command_source == "external_vla"' in wbc
    assert "external_command_provider.update" in wbc
    assert "external_target_watchdog_sec" in arm
    assert "External VLA arm watchdog expired" in arm


def test_real_runner_phase_advances_from_grasp_to_place_navigation():
    sys.path.insert(0, str(ROOT / "scripts"))
    from vla_real_client import _next_phase

    assert _next_phase("nav_pick", "nav") == "nav_pick"
    assert _next_phase("nav_pick", "grasp") == "grasp"
    assert _next_phase("grasp", "nav") == "nav_place"
    assert _next_phase("nav_place", "place") == "place"
