"""Pure-Python VLA adapters. This module never publishes ROS or CAN commands."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from .base_command_provider import BaseCommand
from .vla_command_contract import ArmTargetBase, parse_arm_target, parse_nav_waypoint


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class WaypointVelocityConfig:
    position_tolerance_m: float = 0.08
    yaw_tolerance_rad: float = 0.15
    translation_gain: float = 0.8
    yaw_gain: float = 1.2
    max_vx_mps: float = 0.20
    max_vy_mps: float = 0.10
    max_yaw_rate_rps: float = 0.30


@dataclass(frozen=True)
class WaypointVelocityResult:
    command: BaseCommand
    reached: bool


class WaypointVelocityAdapter:
    """Convert one body-frame waypoint error to a bounded velocity proposal."""

    def __init__(self, config: Optional[WaypointVelocityConfig] = None) -> None:
        self.config = config or WaypointVelocityConfig()

    def compute(self, waypoint: Sequence[float], *, stamp: float) -> WaypointVelocityResult:
        target = parse_nav_waypoint(waypoint)
        reached = (
            math.hypot(target.dx, target.dy) <= self.config.position_tolerance_m
            and abs(target.dyaw) <= self.config.yaw_tolerance_rad
        )
        if reached:
            values = (0.0, 0.0, 0.0)
        else:
            values = (
                _clip(self.config.translation_gain * target.dx, self.config.max_vx_mps),
                _clip(self.config.translation_gain * target.dy, self.config.max_vy_mps),
                _clip(self.config.yaw_gain * target.dyaw, self.config.max_yaw_rate_rps),
            )
        return WaypointVelocityResult(
            command=BaseCommand(*values, stamp=float(stamp), source="vla_waypoint_adapter"),
            reached=reached,
        )


@dataclass(frozen=True)
class ArmTargetSafetyConfig:
    min_xyz: tuple[float, float, float] = (0.05, -0.45, -0.10)
    max_xyz: tuple[float, float, float] = (0.70, 0.45, 0.55)
    max_translation_step_m: float = 0.08
    max_rotation_step_rad: float = 0.35


@dataclass(frozen=True)
class ArmTargetSafetyResult:
    accepted: bool
    reason: str
    target: Optional[ArmTargetBase]


class ArmTargetSafetyGate:
    """Check a base-frame TCP proposal before any IK or hardware adapter."""

    def __init__(self, config: Optional[ArmTargetSafetyConfig] = None) -> None:
        self.config = config or ArmTargetSafetyConfig()

    def evaluate(
        self,
        target_values: Sequence[float],
        *,
        current_values: Sequence[float],
    ) -> ArmTargetSafetyResult:
        target = parse_arm_target(target_values)
        current = parse_arm_target(current_values)
        target_xyz = (target.x, target.y, target.z)
        for index, axis in enumerate("xyz"):
            if not self.config.min_xyz[index] <= target_xyz[index] <= self.config.max_xyz[index]:
                return ArmTargetSafetyResult(False, f"tcp_{axis}_outside_workspace", target)
        translation = math.sqrt(
            (target.x - current.x) ** 2
            + (target.y - current.y) ** 2
            + (target.z - current.z) ** 2
        )
        if translation > self.config.max_translation_step_m:
            return ArmTargetSafetyResult(False, "tcp_translation_step_too_large", target)
        rotation = max(
            abs(_wrap_angle(target.roll - current.roll)),
            abs(_wrap_angle(target.pitch - current.pitch)),
            abs(_wrap_angle(target.yaw - current.yaw)),
        )
        if rotation > self.config.max_rotation_step_rad:
            return ArmTargetSafetyResult(False, "tcp_rotation_step_too_large", target)
        return ArmTargetSafetyResult(True, "accepted_for_planning_only", target)


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def model_arm_target_to_pose7(
    target: Sequence[float], *, gripper_width_m: float = 0.088
) -> Tuple[Tuple[float, ...], float]:
    values = parse_arm_target(target)
    roll, pitch, yaw = values.roll, values.pitch, values.yaw
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    pose7 = (
        values.x,
        values.y,
        values.z,
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    return pose7, values.gripper * float(gripper_width_m)


def pose7_to_model_arm_target(
    pose7: Sequence[float],
    gripper_width_m: float,
    *,
    max_gripper_width_m: float = 0.088,
) -> Tuple[float, ...]:
    values = tuple(float(value) for value in pose7)
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError("pose7 must be a finite 7-vector")
    qw, qx, qy, qz = values[3:]
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1.0e-8:
        raise ValueError("pose7 quaternion has near-zero norm")
    qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    normalized_gripper = float(gripper_width_m) / float(max_gripper_width_m)
    return values[:3] + (roll, pitch, yaw, normalized_gripper)
