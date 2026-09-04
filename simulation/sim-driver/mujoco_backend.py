"""Headless MuJoCo backend for the simulated G1 Driver.

The backend provides real MuJoCo joint, IMU and contact telemetry.  Until a
locomotion policy passes the complete official sim2sim transition, it exposes
no walking actuator: the virtual base servo is only a standing stabilizer.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import Callable, Iterable

import mujoco
import numpy as np

from state import FAULT_MODES


BACKEND_NAME = "mujoco_g1_29dof"
STABILIZATION_MODE = "joint_position_servo_with_virtual_base_stabilization"
WAVE_MIN_DURATION_SECONDS = 4.0
WAVE_MAX_DURATION_SECONDS = 10.0
WAVE_RAISE_SECONDS = 1.0
WAVE_LOWER_SECONDS = 1.0
WAVE_FREQUENCY_HZ = 1.15
WAVE_KEY_TARGETS = {
    "left_shoulder_pitch": -1.0,
    "left_shoulder_roll": 1.7,
    "left_shoulder_yaw": 0.5,
    "left_elbow": 0.8,
    "left_wrist_roll": 0.0,
    "left_wrist_pitch": -0.1,
    "left_wrist_yaw": 0.0,
}
STAND_TARGETS = {
    "left_hip_pitch": -0.1,
    "left_knee": 0.3,
    "left_ankle_pitch": -0.2,
    "right_hip_pitch": -0.1,
    "right_knee": 0.3,
    "right_ankle_pitch": -0.2,
    "left_shoulder_pitch": 0.2,
    "left_elbow": 0.9,
    "right_shoulder_pitch": 0.2,
    "right_elbow": 0.9,
}


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalise_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _quat_to_rpy(quaternion: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = (float(value) for value in quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = _clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class MujocoSimulationState:
    """Thread-safe G1 physics state using the locked official MJCF model."""

    backend_name = BACKEND_NAME

    def __init__(
        self,
        model_path: str | Path,
        *,
        clock: Callable[[], float] = time.monotonic,
        seed: int = 7,
        timestep: float = 0.002,
    ):
        self._clock = clock
        self._lock = threading.RLock()
        self._seed = int(seed)
        self._model_path = Path(model_path).resolve()
        if not self._model_path.is_file():
            raise FileNotFoundError(f"MuJoCo G1 scene not found: {self._model_path}")
        self._model = mujoco.MjModel.from_xml_path(str(self._model_path))
        self._model.opt.timestep = float(timestep)
        self._data = mujoco.MjData(self._model)
        self._timestep = float(self._model.opt.timestep)
        self._last_tick = float(clock())
        self._push_deadline: float | None = None
        self._push_force = np.zeros(2, dtype=np.float64)
        self._target_position = np.zeros(2, dtype=np.float64)
        self._target_yaw = 0.0
        self._paused = False
        self._fault_mode = "none"
        self._balance_assist = True
        self._sequence = 0

        self._actuator_names = [
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            for index in range(self._model.nu)
        ]
        self._actuator_index = {
            name: index for index, name in enumerate(self._actuator_names) if name
        }
        self._gesture = "idle"
        self._gesture_started_sim_time = 0.0
        self._gesture_duration = 0.0
        self._gesture_deadline_sim_time: float | None = None
        self._joint_ids = np.asarray(self._model.actuator_trnid[:, 0], dtype=np.int32)
        self._qpos_ids = np.asarray(
            [self._model.jnt_qposadr[joint_id] for joint_id in self._joint_ids],
            dtype=np.int32,
        )
        self._qvel_ids = np.asarray(
            [self._model.jnt_dofadr[joint_id] for joint_id in self._joint_ids],
            dtype=np.int32,
        )
        self._stand_target = np.asarray(
            [STAND_TARGETS.get(name, 0.0) for name in self._actuator_names],
            dtype=np.float64,
        )
        self._kp = np.full(self._model.nu, 20.0, dtype=np.float64)
        self._kd = np.full(self._model.nu, 1.0, dtype=np.float64)
        for index, name in enumerate(self._actuator_names):
            if any(token in name for token in ("hip", "knee")):
                self._kp[index], self._kd[index] = 80.0, 3.0
            elif "ankle" in name:
                self._kp[index], self._kd[index] = 50.0, 2.0
            elif "waist" in name:
                self._kp[index], self._kd[index] = 50.0, 2.0

        self._pelvis_body = self._model.body("pelvis").id
        self._torso_body = self._model.body("torso_link").id
        self._left_foot_bodies = self._descendants("left_ankle_roll_link")
        self._right_foot_bodies = self._descendants("right_ankle_roll_link")
        self._contact_force = np.zeros(6, dtype=np.float64)
        self._reset_data()

    def _descendants(self, root_name: str) -> set[int]:
        root = self._model.body(root_name).id
        result = {root}
        changed = True
        while changed:
            changed = False
            for body_id in range(1, self._model.nbody):
                if body_id not in result and int(self._model.body_parentid[body_id]) in result:
                    result.add(body_id)
                    changed = True
        return result

    def _reset_data(self) -> None:
        self._data = mujoco.MjData(self._model)
        self._data.qpos[self._qpos_ids] = self._stand_target
        mujoco.mj_forward(self._model, self._data)
        self._target_position[:] = self._data.qpos[:2]
        self._target_yaw = 0.0
        self._push_deadline = None
        self._push_force[:] = 0.0
        self._gesture = "idle"
        self._gesture_started_sim_time = 0.0
        self._gesture_duration = 0.0
        self._gesture_deadline_sim_time = None

    def _current_rpy(self) -> tuple[float, float, float]:
        return _quat_to_rpy(np.asarray(self._data.xquat[self._pelvis_body], dtype=np.float64))

    def _joint_target(self) -> np.ndarray:
        target = self._stand_target.copy()
        if self._gesture != "wave":
            return target
        if (
            self._gesture_deadline_sim_time is not None
            and self._data.time + self._timestep / 2.0 >= self._gesture_deadline_sim_time
        ):
            self._gesture = "idle"
            self._gesture_duration = 0.0
            self._gesture_deadline_sim_time = None
            return target

        elapsed = max(0.0, float(self._data.time) - self._gesture_started_sim_time)
        lower_started = self._gesture_duration - WAVE_LOWER_SECONDS
        if elapsed < WAVE_RAISE_SECONDS:
            arm_weight = self._smoothstep(elapsed / WAVE_RAISE_SECONDS)
            wave_weight = 0.0
        elif elapsed < lower_started:
            arm_weight = 1.0
            wave_weight = 1.0
        else:
            lower_progress = self._smoothstep(
                (elapsed - lower_started) / WAVE_LOWER_SECONDS
            )
            arm_weight = 1.0 - lower_progress
            wave_weight = arm_weight

        for name, raised_value in WAVE_KEY_TARGETS.items():
            index = self._actuator_index.get(name)
            if index is not None:
                target[index] += arm_weight * (raised_value - target[index])

        wave_elapsed = max(0.0, elapsed - WAVE_RAISE_SECONDS)
        wave_phase = 2.0 * math.pi * WAVE_FREQUENCY_HZ * wave_elapsed
        for name, amplitude, phase_offset in (
            ("left_wrist_yaw", 0.7, 0.0),
            ("left_wrist_roll", 0.22, math.pi / 2.0),
            ("left_shoulder_yaw", 0.10, 0.0),
        ):
            index = self._actuator_index.get(name)
            if index is not None:
                target[index] += wave_weight * amplitude * math.sin(
                    wave_phase + phase_offset
                )
        return target

    @staticmethod
    def _smoothstep(value: float) -> float:
        bounded = _clamp(value, 0.0, 1.0)
        return bounded * bounded * (3.0 - 2.0 * bounded)

    def _gesture_phase(self) -> str:
        if self._gesture != "wave":
            return "idle"
        elapsed = max(0.0, float(self._data.time) - self._gesture_started_sim_time)
        if elapsed < WAVE_RAISE_SECONDS:
            return "raising"
        if elapsed < self._gesture_duration - WAVE_LOWER_SECONDS:
            return "waving"
        return "lowering"

    def _apply_joint_controller(self) -> None:
        position_error = self._joint_target() - self._data.qpos[self._qpos_ids]
        velocity = self._data.qvel[self._qvel_ids]
        self._data.ctrl[:] = self._kp * position_error - self._kd * velocity

    def _apply_balance_assist(self) -> None:
        self._data.qfrc_applied[:] = 0.0
        if not self._balance_assist:
            return

        position_error = self._target_position - self._data.qpos[:2]
        velocity_error = -self._data.qvel[:2]
        planar_force = 1000.0 * position_error + 200.0 * velocity_error
        self._data.qfrc_applied[:2] = np.clip(planar_force, -400.0, 400.0)

        roll, pitch, current_yaw = self._current_rpy()
        rotation_error = np.asarray(
            [-roll, -pitch, _normalise_yaw(self._target_yaw - current_yaw)],
            dtype=np.float64,
        )
        angular_velocity_error = -self._data.qvel[3:6]
        torque = 500.0 * rotation_error + 50.0 * angular_velocity_error
        self._data.qfrc_applied[3:6] = np.clip(torque, -200.0, 200.0)

    def _apply_push(self, now: float) -> None:
        if self._push_deadline is not None and now < self._push_deadline:
            self._data.qfrc_applied[:2] += self._push_force
        else:
            self._push_deadline = None
            self._push_force[:] = 0.0

    def command_wave(self, duration: float = 4.0) -> dict:
        value = float(duration)
        if not math.isfinite(value):
            raise ValueError("wave duration must be a finite number")
        with self._lock:
            bounded_duration = _clamp(
                value,
                WAVE_MIN_DURATION_SECONDS,
                WAVE_MAX_DURATION_SECONDS,
            )
            self._gesture = "wave"
            self._gesture_started_sim_time = float(self._data.time)
            self._gesture_duration = bounded_duration
            self._gesture_deadline_sim_time = self._gesture_started_sim_time + bounded_duration
            return {
                "state": "running",
                "gesture": self._gesture,
                "duration": bounded_duration,
                "control_mode": "mujoco_joint_position_servo",
                "motion_semantics": "raise_left_arm_wave_wrist_then_lower",
            }

    def stop_gesture(self) -> dict:
        with self._lock:
            self._gesture = "idle"
            self._gesture_duration = 0.0
            self._gesture_deadline_sim_time = None
            return {"state": "idle", "gesture": "idle"}

    def reset(self, *, seed: int | None = None) -> dict:
        with self._lock:
            if seed is not None:
                self._seed = int(seed)
            self._paused = False
            self._fault_mode = "none"
            self._balance_assist = True
            self._sequence = 0
            self._reset_data()
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

    def set_balance_assist(self, enabled: bool) -> dict:
        with self._lock:
            self._balance_assist = bool(enabled)
            if self._balance_assist:
                self._target_position[:] = self._data.qpos[:2]
                self._target_yaw = self._current_rpy()[2]
            return {
                "state": "configured",
                "balance_assist": self._balance_assist,
                "assist_type": "virtual_base_pose_servo" if self._balance_assist else "off",
            }

    def apply_push(self, fx: float, fy: float, duration: float = 0.2) -> dict:
        values = (float(fx), float(fy), float(duration))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("push parameters must be finite numbers")
        with self._lock:
            self._push_force[:] = (
                _clamp(values[0], -500.0, 500.0),
                _clamp(values[1], -500.0, 500.0),
            )
            bounded_duration = _clamp(values[2], 0.02, 1.0)
            self._push_deadline = float(self._clock()) + bounded_duration
            return {
                "state": "pushing",
                "force_n": [float(value) for value in self._push_force],
                "duration": bounded_duration,
            }

    def step(self, now: float | None = None) -> dict:
        with self._lock:
            current = float(self._clock() if now is None else now)
            elapsed = _clamp(current - self._last_tick, self._timestep, 0.1)
            self._last_tick = current

            if self._paused:
                self._sequence += 1
                return self.snapshot()

            substeps = max(1, min(50, int(round(elapsed / self._timestep))))
            for _ in range(substeps):
                self._apply_joint_controller()
                self._apply_balance_assist()
                self._apply_push(current)
                mujoco.mj_step(self._model, self._data)
            if (
                self._gesture_deadline_sim_time is not None
                and self._data.time + self._timestep / 2.0 >= self._gesture_deadline_sim_time
            ):
                self._gesture = "idle"
                self._gesture_duration = 0.0
                self._gesture_deadline_sim_time = None
            self._sequence += 1
            return self.snapshot()

    def _contact_forces(self) -> tuple[float, float]:
        totals = [0.0, 0.0]
        for index in range(self._data.ncon):
            contact = self._data.contact[index]
            bodies = {
                int(self._model.geom_bodyid[contact.geom1]),
                int(self._model.geom_bodyid[contact.geom2]),
            }
            mujoco.mj_contactForce(self._model, self._data, index, self._contact_force)
            if bodies & self._left_foot_bodies:
                totals[0] += abs(float(self._contact_force[0]))
            if bodies & self._right_foot_bodies:
                totals[1] += abs(float(self._contact_force[0]))
        return totals[0], totals[1]

    def _balance_snapshot(self, contact_forces: tuple[float, float]) -> dict:
        pelvis_height = float(self._data.xpos[self._pelvis_body][2])
        torso_up = float(self._data.xmat[self._torso_body].reshape(3, 3)[2, 2])
        fallen = pelvis_height < 0.45 or torso_up < 0.5
        stable = (
            not fallen
            and pelvis_height > 0.65
            and torso_up > 0.9
            and min(contact_forces) > 20.0
        )
        if fallen:
            state = "fallen"
        elif stable:
            state = "stable"
        elif self._data.time < 0.5:
            state = "settling"
        else:
            state = "unstable"
        return {
            "state": state,
            "fallen": fallen,
            "stable": stable,
            "pelvis_height_m": round(pelvis_height, 6),
            "torso_up_dot": round(torso_up, 6),
        }

    def snapshot(self) -> dict:
        with self._lock:
            _, _, yaw = self._current_rpy()
            return {
                "schema_version": "2.0",
                "simulation": True,
                "simulation_backend": BACKEND_NAME,
                "seed": self._seed,
                "sequence": self._sequence,
                "sim_time": round(float(self._data.time), 6),
                "paused": self._paused,
                "fault_mode": self._fault_mode,
                "gesture": self._gesture,
                "gesture_phase": self._gesture_phase(),
                "pose": {
                    "x": round(float(self._data.qpos[0]), 6),
                    "y": round(float(self._data.qpos[1]), 6),
                    "yaw": round(yaw, 6),
                },
                "velocity": {
                    "vx": round(float(self._data.qvel[0]), 6),
                    "vy": round(float(self._data.qvel[1]), 6),
                    "vyaw": round(float(self._data.qvel[5]), 6),
                },
                "command_velocity": {
                    "vx": 0.0,
                    "vy": 0.0,
                    "vyaw": 0.0,
                },
            }

    def loco_snapshot(self) -> dict:
        with self._lock:
            state = self.snapshot()
            contact_forces = self._contact_forces()
            balance = self._balance_snapshot(contact_forces)
            quaternion = [round(float(value), 6) for value in self._data.sensor("imu_quat").data]
            gyro = [round(float(value), 6) for value in self._data.sensor("imu_gyro").data]
            acceleration = [round(float(value), 6) for value in self._data.sensor("imu_acc").data]
            return {
                **state,
                "robot_morphology": "humanoid_biped",
                "physical_telemetry": {
                    "valid": True,
                    "source": "MuJoCo 3.3.6 contact and rigid-body solver",
                    "balance_assist": self._balance_assist,
                    "assist_type": "virtual_base_pose_servo" if self._balance_assist else "off",
                    "autonomous_balance": False,
                    "gait_valid": False,
                    "limitations": (
                        "Virtual base stabilization is standing assistance only; "
                        "no locomotion actuator is exposed until the official "
                        "29DoF sim2sim policy transition passes acceptance."
                    ),
                },
                "control_mode": STABILIZATION_MODE,
                "mode": 0,
                "gait_type": 0,
                "gait_valid": False,
                "body_height": balance["pelvis_height_m"],
                "position": [
                    state["pose"]["x"],
                    state["pose"]["y"],
                    balance["pelvis_height_m"],
                ],
                "velocity_vector": [state["velocity"]["vx"], state["velocity"]["vy"], 0.0],
                "yaw_speed": state["velocity"]["vyaw"],
                "contact_forces_n": {
                    "left_foot": round(contact_forces[0], 3),
                    "right_foot": round(contact_forces[1], 3),
                },
                "foot_force": [round(contact_forces[0], 3), round(contact_forces[1], 3)],
                "foot_force_layout": ["left_foot_total", "right_foot_total"],
                "foot_force_valid": True,
                "balance": balance,
                "imu": {
                    "quaternion": quaternion,
                    "gyroscope": gyro,
                    "accelerometer": acceleration,
                    "rpy": [round(value, 6) for value in self._current_rpy()],
                },
            }

    def imu_snapshot(self) -> dict:
        with self._lock:
            loco = self.loco_snapshot()
            return {
                "schema_version": "2.0",
                "simulation": True,
                "simulation_backend": BACKEND_NAME,
                "sequence": loco["sequence"],
                **loco["imu"],
            }

    def battery_snapshot(self) -> dict:
        with self._lock:
            state = self.snapshot()
            soc = max(5.0, 92.0 - state["sim_time"] / 3600.0 * 4.0)
            return {
                "schema_version": "2.0",
                "simulation": True,
                "simulation_backend": BACKEND_NAME,
                "fixture": True,
                "sequence": state["sequence"],
                "soc_percent": round(soc, 3),
                "soh_percent": 98.0,
                "voltage_v": 75.6,
                "current_a": -1.2,
                "temperature_c": 31.0,
            }

    def joints_snapshot(self, names: Iterable[str]) -> dict:
        with self._lock:
            joints = []
            for index, name in enumerate(names):
                joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id < 0:
                    joints.append(
                        {"idx": index, "name": name, "q": 0.0, "dq": 0.0, "tau": 0.0, "valid": False}
                    )
                    continue
                qpos_id = int(self._model.jnt_qposadr[joint_id])
                qvel_id = int(self._model.jnt_dofadr[joint_id])
                joints.append(
                    {
                        "idx": index,
                        "name": name,
                        "q": round(float(self._data.qpos[qpos_id]), 6),
                        "dq": round(float(self._data.qvel[qvel_id]), 6),
                        "tau": round(float(self._data.qfrc_actuator[qvel_id]), 6),
                        "valid": True,
                    }
                )
            return {
                "schema_version": "2.0",
                "simulation": True,
                "simulation_backend": BACKEND_NAME,
                "sequence": self._sequence,
                "joints": joints,
                "imu_quat": [round(float(value), 6) for value in self._data.sensor("imu_quat").data],
            }
