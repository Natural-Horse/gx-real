"""与 StarVLA/pct_scene 一致的 VLA 动作语义；本模块不接触 CAN。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


ACTION_DIM = 10
NAV_DIM = 3
ARM_DIM = 7
ACTION_NAMES = (
    "dx_body",
    "dy_body",
    "dyaw",
    "tcp_x_base",
    "tcp_y_base",
    "tcp_z_base",
    "roll_base",
    "pitch_base",
    "yaw_base",
    "gripper",
)


class VLACommandError(ValueError):
    pass


@dataclass(frozen=True)
class NavWaypointBody:
    dx: float
    dy: float
    dyaw: float


@dataclass(frozen=True)
class ArmTargetBase:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    gripper: float


def _finite_vector(values: Sequence[float], size: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise VLACommandError(f"{name} must contain {size} values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise VLACommandError(f"{name} contains non-finite values")
    return result


def parse_nav_waypoint(values: Sequence[float]) -> NavWaypointBody:
    dx, dy, dyaw = _finite_vector(values, NAV_DIM, "nav waypoint")
    return NavWaypointBody(dx=dx, dy=dy, dyaw=dyaw)


def parse_arm_target(values: Sequence[float]) -> ArmTargetBase:
    x, y, z, roll, pitch, yaw, gripper = _finite_vector(values, ARM_DIM, "arm target")
    if not 0.0 <= gripper <= 1.0:
        raise VLACommandError(f"arm target gripper must be in [0,1], got {gripper}")
    return ArmTargetBase(x, y, z, roll, pitch, yaw, gripper)


def split_route_action(values: Sequence[float], route: str):
    action = _finite_vector(values, ACTION_DIM, "route action")
    route = str(route).strip().lower()
    if route == "nav":
        return parse_nav_waypoint(action[:NAV_DIM])
    if route in {"grasp", "place"}:
        return parse_arm_target(action[NAV_DIM:])
    if route in {"done", "recover"}:
        return None
    raise VLACommandError(f"unsupported route={route!r}")
