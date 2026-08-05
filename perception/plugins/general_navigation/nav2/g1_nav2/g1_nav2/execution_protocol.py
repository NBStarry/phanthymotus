"""N5 structured velocity proposal and fail-closed Driver gate reference.

This module deliberately has no ROS or Unitree dependency.  The Nav2 companion
uses :func:`build_velocity_proposal` to wrap its anonymous Twist output with a
navigation identity and a short TTL.  The real G1 Driver remains responsible
for owning an :class:`ExecutorGate` equivalent and for translating an accepted
velocity into bounded-duration high-level motion calls.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re
import time
from typing import Callable


SCHEMA_VERSION = 1
VELOCITY_PROPOSAL_SCHEMA = "phanthy.navigation.velocity_proposal.v1"
VELOCITY_PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
EXECUTOR_STATUS_TOPIC = "/ubuntu/navigation/executor/status"

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
_MOTION_STATUSES = {"planning", "navigating", "replanning", "running", "active"}
_IDLE_STATUSES = {"paused"}
_TERMINAL_STATUSES = {
    "arrived",
    "cancelled",
    "stopped",
    "error",
    "aborted",
    "rejected",
}


class ProtocolError(ValueError):
    """Malformed or unsafe data-plane proposal."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


class GateError(RuntimeError):
    """Invalid owner control-plane operation."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


class GateState(str, Enum):
    UNARMED = "unarmed"
    ARMED_IDLE = "armed_idle"
    EXECUTING = "executing"
    STOPPING = "stopping"
    ESTOPPED = "estopped"


@dataclass(frozen=True)
class Velocity:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @classmethod
    def zero(cls) -> "Velocity":
        return cls()

    def is_zero(self, *, tolerance: float = 1e-9) -> bool:
        return (
            abs(self.x) <= tolerance
            and abs(self.y) <= tolerance
            and abs(self.yaw) <= tolerance
        )

    def as_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw}


@dataclass(frozen=True)
class VelocityLimits:
    min_x: float = -0.05
    max_x: float = 0.15
    max_abs_y: float = 0.12
    max_abs_yaw: float = 0.35
    max_planar_speed: float = 0.18

    def validate(self, velocity: Velocity) -> None:
        values = (velocity.x, velocity.y, velocity.yaw)
        if any(not math.isfinite(value) for value in values):
            raise ProtocolError("non_finite_velocity", "velocity must be finite")
        if not self.min_x <= velocity.x <= self.max_x:
            raise ProtocolError(
                "velocity_limit",
                f"x must be within [{self.min_x}, {self.max_x}] m/s",
            )
        if abs(velocity.y) > self.max_abs_y:
            raise ProtocolError(
                "velocity_limit",
                f"abs(y) must not exceed {self.max_abs_y} m/s",
            )
        if abs(velocity.yaw) > self.max_abs_yaw:
            raise ProtocolError(
                "velocity_limit",
                f"abs(yaw) must not exceed {self.max_abs_yaw} rad/s",
            )
        if math.hypot(velocity.x, velocity.y) > self.max_planar_speed:
            raise ProtocolError(
                "velocity_limit",
                f"planar speed must not exceed {self.max_planar_speed} m/s",
            )


DEFAULT_VELOCITY_LIMITS = VelocityLimits()


def _identifier(value, field: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ProtocolError(
            "invalid_identifier",
            f"{field} must match {_IDENTIFIER.pattern}",
        )
    return value


def _positive_int(value, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ProtocolError("invalid_integer", f"{field} must be a positive integer")
    return value


def _finite_number(value, field: str) -> float:
    if isinstance(value, bool):
        raise ProtocolError("invalid_number", f"{field} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(
            "invalid_number", f"{field} must be a finite number"
        ) from exc
    if not math.isfinite(number):
        raise ProtocolError("invalid_number", f"{field} must be a finite number")
    return number


@dataclass(frozen=True)
class VelocityProposal:
    nav_id: str
    sequence: int
    ttl_ms: int
    issued_at_unix_ms: int
    navigation_status: str
    velocity: Velocity
    reason: str | None = None

    @classmethod
    def from_payload(
        cls,
        payload: dict,
        *,
        limits: VelocityLimits = DEFAULT_VELOCITY_LIMITS,
        max_ttl_ms: int = 250,
    ) -> "VelocityProposal":
        if not isinstance(payload, dict):
            raise ProtocolError("invalid_payload", "proposal must be a JSON object")
        if payload.get("schema") != VELOCITY_PROPOSAL_SCHEMA:
            raise ProtocolError(
                "schema_mismatch",
                f"schema must be {VELOCITY_PROPOSAL_SCHEMA}",
            )
        if payload.get("frame") != "base_link":
            raise ProtocolError("frame_mismatch", "frame must be base_link")
        if payload.get("shadow_only") is not True:
            raise ProtocolError("unsafe_flag", "shadow_only must be true")
        if payload.get("physical_execution") is not False:
            raise ProtocolError("unsafe_flag", "physical_execution must be false")
        if any(
            alias in payload
            for alias in ("status", "navigation_status", "navigation_state")
        ):
            raise ProtocolError(
                "unsupported_navigation_status_field",
                "nav_status is the only supported navigation status field",
            )

        nav_id = _identifier(payload.get("nav_id"), "nav_id")
        sequence = _positive_int(payload.get("sequence"), "sequence")
        ttl_ms = _positive_int(payload.get("ttl_ms"), "ttl_ms")
        if ttl_ms > max_ttl_ms:
            raise ProtocolError(
                "ttl_limit", f"ttl_ms must not exceed {max_ttl_ms}"
            )
        issued_at_unix_ms = _positive_int(
            payload.get("issued_at_unix_ms"), "issued_at_unix_ms"
        )
        navigation_status = payload.get("nav_status")
        allowed_statuses = _MOTION_STATUSES | _IDLE_STATUSES | _TERMINAL_STATUSES
        if navigation_status not in allowed_statuses:
            raise ProtocolError(
                "invalid_navigation_status",
                f"unsupported nav_status: {navigation_status}",
            )

        raw_velocity = payload.get("velocity")
        if not isinstance(raw_velocity, dict):
            raise ProtocolError("invalid_velocity", "velocity must be an object")
        velocity = Velocity(
            x=_finite_number(raw_velocity.get("x"), "velocity.x"),
            y=_finite_number(raw_velocity.get("y"), "velocity.y"),
            yaw=_finite_number(raw_velocity.get("yaw"), "velocity.yaw"),
        )
        limits.validate(velocity)
        if navigation_status not in _MOTION_STATUSES and not velocity.is_zero():
            raise ProtocolError(
                "unsafe_navigation_state",
                f"{navigation_status} proposals must carry zero velocity",
            )

        reason = payload.get("reason")
        if reason is not None and (not isinstance(reason, str) or len(reason) > 256):
            raise ProtocolError(
                "invalid_reason", "reason must be a string of at most 256 characters"
            )
        return cls(
            nav_id=nav_id,
            sequence=sequence,
            ttl_ms=ttl_ms,
            issued_at_unix_ms=issued_at_unix_ms,
            navigation_status=navigation_status,
            velocity=velocity,
            reason=reason,
        )

    def as_payload(self) -> dict:
        payload = {
            "schema": VELOCITY_PROPOSAL_SCHEMA,
            "nav_id": self.nav_id,
            "sequence": self.sequence,
            "ttl_ms": self.ttl_ms,
            "issued_at_unix_ms": self.issued_at_unix_ms,
            "frame": "base_link",
            "nav_status": self.navigation_status,
            "velocity": self.velocity.as_dict(),
            "shadow_only": True,
            "physical_execution": False,
        }
        if self.reason:
            payload["reason"] = self.reason
        return payload


def build_velocity_proposal(
    *,
    nav_id: str,
    sequence: int,
    ttl_ms: int,
    navigation_status: str,
    velocity: Velocity,
    reason: str | None = None,
    issued_at_unix_ms: int | None = None,
) -> dict:
    """Build and self-validate one N5 data-plane proposal."""

    proposal = VelocityProposal(
        nav_id=nav_id,
        sequence=sequence,
        ttl_ms=ttl_ms,
        issued_at_unix_ms=issued_at_unix_ms or time.time_ns() // 1_000_000,
        navigation_status=navigation_status,
        velocity=velocity,
        reason=reason,
    )
    payload = proposal.as_payload()
    VelocityProposal.from_payload(payload)
    return payload


@dataclass(frozen=True)
class RuntimeHealth:
    main_control_ready: bool
    estop_clear: bool
    odom_age_ms: float
    scan_age_ms: float
    nav2_status_age_ms: float


@dataclass(frozen=True)
class GatePolicy:
    min_lease_ms: int = 500
    max_lease_ms: int = 5000
    heartbeat_timeout_ms: int = 1000
    max_proposal_ttl_ms: int = 250
    max_odom_age_ms: int = 500
    max_scan_age_ms: int = 500
    max_nav2_status_age_ms: int = 1500
    velocity_limits: VelocityLimits = DEFAULT_VELOCITY_LIMITS


@dataclass(frozen=True)
class GateDecision:
    accepted: bool
    state: GateState
    velocity: Velocity
    reason: str
    must_send_zero: bool


class ExecutorGate:
    """Executable N5 state-machine reference for the Driver owner.

    Owner control methods are intentionally direct calls rather than a ROS
    command topic.  The real Driver must expose them only through its trusted
    capability/authentication boundary; the public Perception card must never
    be able to arm this gate.
    """

    def __init__(
        self,
        *,
        policy: GatePolicy = GatePolicy(),
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._policy = policy
        self._clock = clock
        self._state = GateState.UNARMED
        self._velocity = Velocity.zero()
        self._owner_id: str | None = None
        self._lease_id: str | None = None
        self._nav_id: str | None = None
        self._lease_expires_at: float | None = None
        self._last_heartbeat_at: float | None = None
        self._proposal_expires_at: float | None = None
        self._last_sequence = 0
        self._stop_reason = "startup_unarmed"
        self._estop_pending = False

    @property
    def state(self) -> GateState:
        return self._state

    def snapshot(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "event": "executor_status",
            "state": self._state.value,
            "owner_id": self._owner_id,
            "lease_id": self._lease_id,
            "nav_id": self._nav_id,
            "last_sequence": self._last_sequence,
            "velocity": self._velocity.as_dict(),
            "stop_reason": self._stop_reason,
        }

    def arm(
        self,
        *,
        owner_id: str,
        lease_id: str,
        nav_id: str,
        lease_duration_ms: int,
        health: RuntimeHealth,
    ) -> GateDecision:
        if self._state is not GateState.UNARMED:
            raise GateError("gate_not_unarmed", f"gate is {self._state.value}")
        self._validate_control_identifier(owner_id, "owner_id")
        self._validate_control_identifier(lease_id, "lease_id")
        self._validate_control_identifier(nav_id, "nav_id")
        self._validate_lease_duration(lease_duration_ms)
        violation = self._health_violation(health)
        if violation:
            raise GateError("runtime_not_ready", violation)

        now = self._clock()
        self._owner_id = owner_id
        self._lease_id = lease_id
        self._nav_id = nav_id
        self._lease_expires_at = now + lease_duration_ms / 1000.0
        self._last_heartbeat_at = now
        self._proposal_expires_at = None
        self._last_sequence = 0
        self._velocity = Velocity.zero()
        self._stop_reason = ""
        self._state = GateState.ARMED_IDLE
        return self._decision(True, "armed")

    def renew(
        self,
        *,
        owner_id: str,
        lease_id: str,
        lease_duration_ms: int,
        health: RuntimeHealth,
    ) -> GateDecision:
        self._require_active_lease(owner_id=owner_id, lease_id=lease_id)
        self._validate_lease_duration(lease_duration_ms)
        violation = self._health_violation(health)
        if violation:
            return self._stop_for_health(violation)
        now = self._clock()
        self._lease_expires_at = now + lease_duration_ms / 1000.0
        self._last_heartbeat_at = now
        return self._decision(True, "renewed")

    def ingest(self, payload: dict, *, health: RuntimeHealth) -> GateDecision:
        if self._state not in {GateState.ARMED_IDLE, GateState.EXECUTING}:
            return self._decision(False, "gate_not_armed")

        violation = self._health_violation(health)
        if violation:
            return self._stop_for_health(violation)
        timeout = self._lease_timeout_reason(self._clock())
        if timeout:
            return self._begin_stop(timeout)

        try:
            proposal = VelocityProposal.from_payload(
                payload,
                limits=self._policy.velocity_limits,
                max_ttl_ms=self._policy.max_proposal_ttl_ms,
            )
        except ProtocolError as exc:
            return self._begin_stop(f"invalid_proposal:{exc.code}")

        if proposal.nav_id != self._nav_id:
            return self._begin_stop("nav_id_mismatch")
        if proposal.sequence <= self._last_sequence:
            return self._begin_stop("sequence_replay")

        now = self._clock()
        self._last_sequence = proposal.sequence
        self._proposal_expires_at = now + proposal.ttl_ms / 1000.0
        if proposal.navigation_status in _TERMINAL_STATUSES:
            return self._begin_stop(
                proposal.reason or f"navigation_{proposal.navigation_status}"
            )
        if proposal.navigation_status in _IDLE_STATUSES:
            self._velocity = Velocity.zero()
            self._state = GateState.ARMED_IDLE
            return self._decision(True, proposal.reason or "navigation_paused")

        self._velocity = proposal.velocity
        self._state = (
            GateState.ARMED_IDLE
            if proposal.velocity.is_zero()
            else GateState.EXECUTING
        )
        return self._decision(True, proposal.reason or "proposal_accepted")

    def poll(self, *, health: RuntimeHealth) -> GateDecision:
        if self._state in {GateState.UNARMED, GateState.ESTOPPED}:
            return self._decision(False, self._stop_reason)
        if self._state is GateState.STOPPING:
            return self._decision(False, self._stop_reason)

        violation = self._health_violation(health)
        if violation:
            return self._stop_for_health(violation)
        now = self._clock()
        timeout = self._lease_timeout_reason(now)
        if timeout:
            return self._begin_stop(timeout)
        if (
            self._state is GateState.EXECUTING
            and self._proposal_expires_at is not None
            and now >= self._proposal_expires_at
        ):
            return self._begin_stop("proposal_ttl_expired")
        return self._decision(True, "healthy")

    def disarm(self, *, owner_id: str, lease_id: str) -> GateDecision:
        self._require_active_lease(owner_id=owner_id, lease_id=lease_id)
        return self._begin_stop("owner_disarm")

    def estop(self, *, reason: str = "emergency_stop") -> GateDecision:
        if self._state is GateState.ESTOPPED:
            return self._decision(False, self._stop_reason)
        self._estop_pending = True
        return self._begin_stop(reason)

    def acknowledge_stopped(self) -> GateDecision:
        if self._state is not GateState.STOPPING:
            raise GateError("not_stopping", f"gate is {self._state.value}")
        estopped = self._estop_pending
        reason = self._stop_reason
        self._clear_lease()
        self._estop_pending = False
        self._state = GateState.ESTOPPED if estopped else GateState.UNARMED
        self._stop_reason = reason
        return self._decision(True, "stop_acknowledged")

    def reset_estop(
        self,
        *,
        owner_id: str,
        health: RuntimeHealth,
    ) -> GateDecision:
        if self._state is not GateState.ESTOPPED:
            raise GateError("not_estopped", f"gate is {self._state.value}")
        self._validate_control_identifier(owner_id, "owner_id")
        violation = self._health_violation(health)
        if violation:
            raise GateError("runtime_not_ready", violation)
        self._estop_pending = False
        self._state = GateState.UNARMED
        self._stop_reason = "estop_reset"
        return self._decision(True, "estop_reset")

    def _begin_stop(self, reason: str) -> GateDecision:
        self._velocity = Velocity.zero()
        self._proposal_expires_at = None
        self._stop_reason = reason
        self._state = GateState.STOPPING
        return self._decision(False, reason)

    def _stop_for_health(self, violation: str) -> GateDecision:
        if violation == "estop_not_clear":
            self._estop_pending = True
        return self._begin_stop(f"health:{violation}")

    def _clear_lease(self) -> None:
        self._velocity = Velocity.zero()
        self._owner_id = None
        self._lease_id = None
        self._nav_id = None
        self._lease_expires_at = None
        self._last_heartbeat_at = None
        self._proposal_expires_at = None
        self._last_sequence = 0

    def _decision(self, accepted: bool, reason: str) -> GateDecision:
        return GateDecision(
            accepted=accepted,
            state=self._state,
            velocity=self._velocity,
            reason=reason,
            must_send_zero=self._state
            in {GateState.UNARMED, GateState.STOPPING, GateState.ESTOPPED}
            or self._velocity.is_zero(),
        )

    def _lease_timeout_reason(self, now: float) -> str | None:
        if self._lease_expires_at is None or now >= self._lease_expires_at:
            return "lease_expired"
        if (
            self._last_heartbeat_at is None
            or (now - self._last_heartbeat_at) * 1000.0
            >= self._policy.heartbeat_timeout_ms
        ):
            return "heartbeat_timeout"
        return None

    def _health_violation(self, health: RuntimeHealth) -> str | None:
        if not health.main_control_ready:
            return "main_control_not_ready"
        if not health.estop_clear:
            return "estop_not_clear"
        ages = (
            ("odom_stale", health.odom_age_ms, self._policy.max_odom_age_ms),
            ("scan_stale", health.scan_age_ms, self._policy.max_scan_age_ms),
            (
                "nav2_status_stale",
                health.nav2_status_age_ms,
                self._policy.max_nav2_status_age_ms,
            ),
        )
        for reason, age, maximum in ages:
            if isinstance(age, bool) or not isinstance(age, (int, float)):
                return reason
            if not math.isfinite(float(age)) or age < 0 or age > maximum:
                return reason
        return None

    def _require_active_lease(self, *, owner_id: str, lease_id: str) -> None:
        if self._state not in {GateState.ARMED_IDLE, GateState.EXECUTING}:
            raise GateError("gate_not_armed", f"gate is {self._state.value}")
        if owner_id != self._owner_id or lease_id != self._lease_id:
            raise GateError("lease_mismatch", "owner_id or lease_id does not match")

    def _validate_lease_duration(self, lease_duration_ms: int) -> None:
        if isinstance(lease_duration_ms, bool) or not isinstance(lease_duration_ms, int):
            raise GateError("invalid_lease", "lease_duration_ms must be an integer")
        if not self._policy.min_lease_ms <= lease_duration_ms <= self._policy.max_lease_ms:
            raise GateError(
                "invalid_lease",
                "lease_duration_ms must be within "
                f"[{self._policy.min_lease_ms}, {self._policy.max_lease_ms}]",
            )

    @staticmethod
    def _validate_control_identifier(value: str, field: str) -> None:
        try:
            _identifier(value, field)
        except ProtocolError as exc:
            raise GateError(exc.code, str(exc)) from exc
