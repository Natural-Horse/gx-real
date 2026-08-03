"""Loopback-only client for the StarVLA Go2 evaluation protocol."""

from __future__ import annotations

import asyncio
import base64
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple, Union
from urllib.parse import urlparse
import uuid


PROTOCOL_VERSION = "starvla-go2-eval/v2"
ROUTES = frozenset({"nav", "grasp", "place", "done", "recover"})


class VLARemoteError(RuntimeError):
    pass


def require_loopback_endpoint(endpoint: str) -> str:
    parsed = urlparse(str(endpoint))
    if parsed.scheme != "ws" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
        raise VLARemoteError(
            "VLA endpoint must be a loopback ws:// URL reached through an SSH tunnel"
        )
    if parsed.port is None:
        raise VLARemoteError("VLA endpoint must include an explicit port")
    return str(endpoint)


def jpeg_file_payload(path: Union[str, Path]) -> Dict[str, str]:
    image_path = Path(path).expanduser()
    raw = image_path.read_bytes()
    if not raw:
        raise VLARemoteError(f"empty JPEG file: {image_path}")
    if len(raw) > 8 * 1024 * 1024:
        raise VLARemoteError(f"JPEG exceeds 8 MiB: {image_path}")
    return {
        "encoding": "jpeg_base64",
        "data": base64.b64encode(raw).decode("ascii"),
    }


def _finite_vector(values: Sequence[float], size: int, name: str) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise VLARemoteError(f"{name} must contain {size} values")
    if not all(math.isfinite(value) for value in result):
        raise VLARemoteError(f"{name} contains non-finite values")
    return result


@dataclass(frozen=True)
class VLAEvaluationDecision:
    route: str
    subtask: Optional[str] = None
    nav_waypoints: tuple[tuple[float, float, float], ...] = ()
    arm_targets_base: tuple[
        tuple[float, float, float, float, float, float, float], ...
    ] = ()
    route_confidence: Optional[float] = None
    raw_text: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_response(cls, data: Any, *, metadata: Optional[Dict[str, Any]] = None):
        if not isinstance(data, dict):
            raise VLARemoteError("inference response data must be an object")
        route = str(data.get("route", "")).strip().lower()
        if route not in ROUTES:
            raise VLARemoteError(f"unsupported route={route!r}")

        waypoints: list[tuple[float, float, float]] = []
        raw_waypoints = data.get("nav_waypoints")
        if route == "nav":
            if not isinstance(raw_waypoints, list) or not raw_waypoints:
                raise VLARemoteError("NAV response has no nav_waypoints")
            if len(raw_waypoints) > 32:
                raise VLARemoteError("NAV response exceeds 32 waypoints")
            waypoints = [
                _finite_vector(point, 3, f"nav_waypoints[{index}]")
                for index, point in enumerate(raw_waypoints)
            ]

        arm_targets: list[tuple[float, float, float, float, float, float, float]] = []
        raw_arm_targets = data.get("arm_targets_base")
        if raw_arm_targets is not None:
            if not isinstance(raw_arm_targets, list) or not raw_arm_targets:
                raise VLARemoteError("arm_targets_base must be a non-empty list")
            if len(raw_arm_targets) > 32:
                raise VLARemoteError("arm_targets_base exceeds 32 targets")
            for index, target in enumerate(raw_arm_targets):
                values = _finite_vector(target, 7, f"arm_targets_base[{index}]")
                if not 0.0 <= values[-1] <= 1.0:
                    raise VLARemoteError(
                        f"arm_targets_base[{index}] gripper must be in [0,1]"
                    )
                arm_targets.append(values)

        confidence = data.get("route_confidence")
        if confidence is not None:
            confidence = float(confidence)
            if not math.isfinite(confidence):
                raise VLARemoteError("route_confidence is non-finite")
        return cls(
            route=route,
            subtask=None if data.get("subtask") is None else str(data["subtask"]),
            nav_waypoints=tuple(waypoints),
            arm_targets_base=tuple(arm_targets),
            route_confidence=confidence,
            raw_text=str(data.get("raw_text", "")),
            metadata=dict(metadata or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "route": self.route,
            "subtask": self.subtask,
            "nav_waypoints": [list(point) for point in self.nav_waypoints],
            "arm_targets_base": [list(target) for target in self.arm_targets_base],
            "route_confidence": self.route_confidence,
            "raw_text": self.raw_text,
            **self.metadata,
        }


@dataclass(frozen=True)
class VLARemoteClientConfig:
    endpoint: str = "ws://127.0.0.1:10093"
    connect_timeout_s: float = 10.0
    response_timeout_s: float = 120.0
    max_message_bytes: int = 20 * 1024 * 1024

    def __post_init__(self) -> None:
        require_loopback_endpoint(self.endpoint)
        if self.connect_timeout_s <= 0.0 or self.response_timeout_s <= 0.0:
            raise VLARemoteError("VLA timeouts must be positive")


class VLARemoteClient:
    """One-request-per-connection client for shadow and gated live ROS adapters."""

    def __init__(self, config: Optional[VLARemoteClientConfig] = None) -> None:
        self.config = config or VLARemoteClientConfig()

    def health(self) -> dict[str, Any]:
        return self._request("health", {})

    def reset(self, episode_id: str) -> dict[str, Any]:
        return self._request("reset", {"episode_id": str(episode_id)})

    def infer(self, payload: dict[str, Any]) -> VLAEvaluationDecision:
        data = self._request("infer", payload)
        timing = data.pop("remote_timing", None)
        return VLAEvaluationDecision.from_response(
            data,
            metadata={"remote_endpoint": self.config.endpoint, "remote_timing": timing},
        )

    def _request(self, request_type: str, payload: dict[str, Any]) -> dict[str, Any]:
        request_id = uuid.uuid4().hex
        request = {
            "protocol_version": PROTOCOL_VERSION,
            "type": request_type,
            "request_id": request_id,
            "payload": payload,
        }
        return asyncio.run(self._exchange(request, request_id))

    async def _exchange(self, request: dict[str, Any], request_id: str) -> dict[str, Any]:
        try:
            import websockets
        except ImportError as exc:
            raise VLARemoteError("install Python package websockets to use VLA shadow mode") from exc
        try:
            async with websockets.connect(
                self.config.endpoint,
                open_timeout=self.config.connect_timeout_s,
                close_timeout=2.0,
                compression=None,
                max_size=self.config.max_message_bytes,
                ping_interval=None,
            ) as connection:
                await connection.send(json.dumps(request, ensure_ascii=True))
                raw_response = await asyncio.wait_for(
                    connection.recv(), timeout=self.config.response_timeout_s
                )
        except Exception as exc:
            raise VLARemoteError(f"remote VLA request failed: {exc}") from exc
        return self._validate_response(raw_response, request_id)

    @staticmethod
    def _validate_response(raw_response: Any, request_id: str) -> dict[str, Any]:
        if not isinstance(raw_response, str):
            raise VLARemoteError("remote VLA returned a binary frame")
        try:
            response = json.loads(raw_response)
        except json.JSONDecodeError as exc:
            raise VLARemoteError("remote VLA returned invalid JSON") from exc
        if not isinstance(response, dict):
            raise VLARemoteError("remote VLA response must be an object")
        if response.get("protocol_version") != PROTOCOL_VERSION:
            raise VLARemoteError("remote VLA protocol version mismatch")
        if response.get("request_id") != request_id:
            raise VLARemoteError("remote VLA request_id mismatch")
        if response.get("ok") is not True:
            error = response.get("error") or {}
            raise VLARemoteError(str(error.get("message") or "remote VLA failed"))
        data = response.get("data")
        if not isinstance(data, dict):
            raise VLARemoteError("remote VLA response has no data object")
        timing = response.get("timing")
        return {**data, "remote_timing": timing if isinstance(timing, dict) else None}
