"""Deterministic protocol-level state for the simulated G1 driver."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Iterable


FAULT_MODES = ("none", "freeze_motion", "drop_camera", "drop_audio")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalise_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class SimulationState:
    """Thread-safe kinematic state machine.

    This is deliberately not a dynamics simulator. Velocity commands are
    integrated deterministically so MCP, ROS and WebUI contracts can be tested
    before MuJoCo is introduced as the authoritative physics backend.
    """

    def __init__(self, *, clock: Callable[[], float] = time.monotonic, seed: int = 7):
        self._clock = clock
        self._lock = threading.RLock()
        self._seed = int(seed)
        self._last_tick = float(clock())
        self._move_deadline: float | None = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._sim_time = 0.0
        self._sequence = 0
        self._paused = False
        self._fault_mode = "none"

    def command_move(self, vx: float, vy: float, vyaw: float, duration: float = 0.0) -> dict:
        values = (float(vx), float(vy), float(vyaw), float(duration))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("move parameters must be finite numbers")
        with self._lock:
            now = float(self._clock())
            self._vx = _clamp(values[0], -1.0, 1.0)
            self._vy = _clamp(values[1], -1.0, 1.0)
            self._vyaw = _clamp(values[2], -2.0, 2.0)
            self._move_deadline = now + values[3] if values[3] > 0 else None
            return {
                "state": "moving",
                "velocity": [self._vx, self._vy, self._vyaw],
                "duration": max(0.0, values[3]),
            }

    def stop_move(self) -> dict:
        with self._lock:
            self._vx = self._vy = self._vyaw = 0.0
            self._move_deadline = None
            return {"state": "stopped", "velocity": [0.0, 0.0, 0.0]}

    def reset(self, *, seed: int | None = None) -> dict:
        with self._lock:
            if seed is not None:
                self._seed = int(seed)
            self._x = self._y = self._yaw = 0.0
            self._vx = self._vy = self._vyaw = 0.0
            self._sim_time = 0.0
            self._sequence = 0
            self._paused = False
            self._fault_mode = "none"
            self._move_deadline = None
            self._last_tick = float(self._clock())
            return self.snapshot()

    def set_paused(self, paused: bool) -> dict:
        with self._lock:
            self._paused = bool(paused)
            self._last_tick = float(self._clock())
            return {"state": "paused" if self._paused else "running", "paused": self._paused}

    def set_fault(self, mode: str) -> dict:
        if mode not in FAULT_MODES:
            raise ValueError(f"fault mode must be one of {FAULT_MODES}")
        with self._lock:
            self._fault_mode = mode
            return {"state": "configured", "fault_mode": mode}

    def step(self, now: float | None = None) -> dict:
        with self._lock:
            current = float(self._clock() if now is None else now)
            dt = _clamp(current - self._last_tick, 0.0, 0.25)
            self._last_tick = current

            if self._move_deadline is not None and current >= self._move_deadline:
                self._vx = self._vy = self._vyaw = 0.0
                self._move_deadline = None

            if not self._paused and self._fault_mode != "freeze_motion":
                cos_yaw = math.cos(self._yaw)
                sin_yaw = math.sin(self._yaw)
                self._x += (self._vx * cos_yaw - self._vy * sin_yaw) * dt
                self._y += (self._vx * sin_yaw + self._vy * cos_yaw) * dt
                self._yaw = _normalise_yaw(self._yaw + self._vyaw * dt)
                self._sim_time += dt
            self._sequence += 1
            return self.snapshot()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "schema_version": "1.0",
                "simulation": True,
                "seed": self._seed,
                "sequence": self._sequence,
                "sim_time": round(self._sim_time, 6),
                "paused": self._paused,
                "fault_mode": self._fault_mode,
                "pose": {
                    "x": round(self._x, 6),
                    "y": round(self._y, 6),
                    "yaw": round(self._yaw, 6),
                },
                "velocity": {
                    "vx": round(self._vx, 6),
                    "vy": round(self._vy, 6),
                    "vyaw": round(self._vyaw, 6),
                },
            }

    def loco_snapshot(self) -> dict:
        state = self.snapshot()
        yaw = state["pose"]["yaw"]
        moving = any(abs(value) > 1e-6 for value in state["velocity"].values())
        return {
            **state,
            "robot_morphology": "humanoid_biped",
            "simulation_backend": "protocol_only_no_physics",
            "physical_telemetry": {
                "valid": False,
                "reason": (
                    "P1 has no dynamics or contact model; do not infer balance, contact, "
                    "motor load, damage, or hardware safety from this sample."
                ),
            },
            "mode": 3 if moving else 0,
            "gait_type": 1 if moving else 0,
            "body_height": 0.78,
            "position": [state["pose"]["x"], state["pose"]["y"], 0.78],
            "velocity_vector": [state["velocity"]["vx"], state["velocity"]["vy"], 0.0],
            "yaw_speed": state["velocity"]["vyaw"],
            "foot_force": None,
            "foot_force_valid": False,
            "imu": {
                "quaternion": [round(math.cos(yaw / 2.0), 6), 0.0, 0.0, round(math.sin(yaw / 2.0), 6)],
                "gyroscope": [0.0, 0.0, state["velocity"]["vyaw"]],
                "accelerometer": [0.0, 0.0, 9.81],
                "rpy": [0.0, 0.0, yaw],
            },
        }

    def imu_snapshot(self) -> dict:
        loco = self.loco_snapshot()
        return {
            "schema_version": "1.0",
            "simulation": True,
            "sequence": loco["sequence"],
            **loco["imu"],
            "temperature_c": 36.5,
        }

    def battery_snapshot(self) -> dict:
        state = self.snapshot()
        soc = max(5.0, 92.0 - state["sim_time"] / 3600.0 * 4.0)
        return {
            "schema_version": "1.0",
            "simulation": True,
            "sequence": state["sequence"],
            "soc_percent": round(soc, 3),
            "soh_percent": 98.0,
            "voltage_v": 75.6,
            "current_a": -1.2,
            "temperature_c": 31.0,
        }

    def joints_snapshot(self, names: Iterable[str]) -> dict:
        state = self.snapshot()
        speed = math.hypot(state["velocity"]["vx"], state["velocity"]["vy"])
        phase = state["sim_time"] * (2.0 + 3.0 * speed)
        joints = []
        for idx, name in enumerate(names):
            sign = -1.0 if name.startswith("right_") else 1.0
            q = 0.0
            if speed > 1e-6:
                if "hip_pitch" in name:
                    q = sign * 0.22 * math.sin(phase)
                elif "knee" in name:
                    q = 0.32 * max(0.0, math.sin(phase + (math.pi if sign < 0 else 0.0)))
                elif "ankle_pitch" in name:
                    q = -0.12 * math.sin(phase)
                elif "shoulder_pitch" in name:
                    q = -sign * 0.16 * math.sin(phase)
            joints.append({"idx": idx, "name": name, "q": round(q, 6), "dq": 0.0, "tau": 0.0})
        return {
            "schema_version": "1.0",
            "simulation": True,
            "sequence": state["sequence"],
            "joints": joints,
            "imu_quat": self.imu_snapshot()["quaternion"],
        }
