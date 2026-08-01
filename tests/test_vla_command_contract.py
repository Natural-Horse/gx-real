from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "real-wbc"))

from modules.vla_command_contract import (  # noqa: E402
    ArmTargetBase,
    NavWaypointBody,
    VLACommandError,
    split_route_action,
)


def test_route_action_selects_only_owned_dimensions():
    action = [0.2, -0.1, 0.3, 0.35, 0.0, 0.2, 0.0, 0.1, -0.2, 1.0]
    assert isinstance(split_route_action(action, "nav"), NavWaypointBody)
    arm = split_route_action(action, "grasp")
    assert isinstance(arm, ArmTargetBase)
    assert arm.x == pytest.approx(0.35)
    assert arm.gripper == pytest.approx(1.0)


def test_arm_gripper_rejects_out_of_range_value():
    with pytest.raises(VLACommandError, match="gripper"):
        split_route_action([0.0] * 9 + [1.2], "place")
