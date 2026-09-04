#!/usr/bin/env python3
"""Cross-container acceptance for the headless MuJoCo G1 backend."""

from __future__ import annotations

import json
import ssl
import time
from urllib import request as urllib_request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


MCP_URL = "http://sim-driver:15730/mcp"
CORE_URL = "https://127.0.0.1:15678"
BACKEND = "mujoco_g1_29dof"
TOPICS = {
    "imu": "/phanthymotus_sim_g1/state/imu",
    "joints": "/phanthymotus_sim_g1/state/joints",
    "loco_state": "/phanthymotus_sim_g1/loco/state",
}


def rpc(method: str, params: dict | None = None, request_id: int = 1) -> dict:
    payload = json.dumps(
        {"jsonrpc": "2.0", "id": request_id, "method": method, "params": params or {}}
    ).encode()
    req = urllib_request.Request(MCP_URL, data=payload, headers={"Content-Type": "application/json"})
    with urllib_request.urlopen(req, timeout=5) as response:
        return json.loads(response.read())


def call(name: str, arguments: dict) -> dict:
    response = rpc("tools/call", {"name": name, "arguments": arguments})
    if "error" in response:
        raise RuntimeError(f"{name} failed: {response['error']}")
    return json.loads(response["result"]["content"][0]["text"])


def core_registry() -> list[dict]:
    context = ssl._create_unverified_context()
    with urllib_request.urlopen(f"{CORE_URL}/api/mcp", timeout=5, context=context) as response:
        payload = json.loads(response.read())
    assert payload["code"] == 200, payload
    return payload["data"]


class Collector(Node):
    def __init__(self):
        super().__init__("sim_g1_p2_acceptance")
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.imu: list[dict] = []
        self.joints: list[dict] = []
        self.loco: list[dict] = []
        self.create_subscription(String, TOPICS["imu"], lambda msg: self.imu.append(json.loads(msg.data)), qos)
        self.create_subscription(
            String,
            TOPICS["joints"],
            lambda msg: self.joints.append(json.loads(msg.data)),
            qos,
        )
        self.create_subscription(
            String,
            TOPICS["loco_state"],
            lambda msg: self.loco.append(json.loads(msg.data)),
            qos,
        )

    def spin_until(self, predicate, timeout: float, description: str) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.loco and predicate(self.loco[-1]):
                return self.loco[-1]
        latest = self.loco[-1] if self.loco else None
        raise AssertionError(f"timeout waiting for {description}; latest={latest}")


def main() -> None:
    initialize = rpc("initialize")
    assert initialize["result"]["serverInfo"]["name"] == "sim-g1-device-bundle", initialize
    tools = rpc("tools/list")["result"]["tools"]
    tool_map = {tool["name"]: tool for tool in tools}
    assert {"imu", "joints", "loco_state", "gesture", "sim_control", "model"} <= set(tool_map), tool_map
    assert "loco" not in tool_map, tool_map
    assert tool_map["gesture"]["inputSchema"]["properties"]["action"]["enum"] == [
        "wave",
        "stop_wave",
    ], tool_map["gesture"]
    control_actions = tool_map["sim_control"]["inputSchema"]["properties"]["action"]["enum"]
    assert {"set_balance_assist", "push"} <= set(control_actions), control_actions
    assert "oscillate the wrist" in tool_map["gesture"]["description"], tool_map["gesture"]
    rejected_loco = rpc(
        "tools/call",
        {"name": "loco", "arguments": {"action": "move", "vx": 0.1}},
    )
    assert "error" in rejected_loco, rejected_loco
    assert "has not passed sim2sim acceptance" in rejected_loco["error"]["message"], rejected_loco
    print("P2 MCP CONTRACT + UNVALIDATED LOCO REJECTION PASS")

    registry = next(item for item in core_registry() if item.get("server_name") == "sim-g1-device-bundle")
    assert registry["name"] == "Simulated G1 (MuJoCo)", registry
    assert registry["url"] == MCP_URL, registry
    print(f"P2 CORE REGISTRATION PASS id={registry['id']}")

    model = call("model", {})
    assert model["simulation_backend"] == BACKEND, model.keys()
    assert '<robot name="g1_' in model["urdf"]
    for sensor in TOPICS:
        result = call(sensor, {"action": "start"})
        assert result["state"] == "running" and result["simulation_backend"] == BACKEND, result
    assert call("gesture", {"action": "start"})["state"] == "ready"
    assert call("sim_control", {"action": "start"})["state"] == "ready"
    reset = call("sim_control", {"action": "reset", "seed": 23})
    assert reset["simulation_backend"] == BACKEND and reset["command_velocity"] == {
        "vx": 0.0,
        "vy": 0.0,
        "vyaw": 0.0,
    }, reset

    rclpy.init()
    collector = Collector()
    try:
        stable = collector.spin_until(
            lambda item: item["balance"]["state"] == "stable",
            5.0,
            "stable assisted standing",
        )
        assert stable["physical_telemetry"]["valid"] is True, stable
        assert stable["physical_telemetry"]["balance_assist"] is True, stable
        assert stable["physical_telemetry"]["autonomous_balance"] is False, stable
        assert stable["gait_valid"] is False and stable["gait_type"] == 0, stable
        assert stable["control_mode"] == "joint_position_servo_with_virtual_base_stabilization", stable
        assert "no locomotion actuator is exposed" in stable["physical_telemetry"]["limitations"], stable
        assert stable["body_height"] > 0.65 and stable["balance"]["torso_up_dot"] > 0.9, stable
        forces = stable["contact_forces_n"]
        assert forces["left_foot"] > 50.0 and forces["right_foot"] > 50.0, forces

        still_samples = []
        still_deadline = time.monotonic() + 2.0
        while time.monotonic() < still_deadline:
            rclpy.spin_once(collector, timeout_sec=0.1)
            if collector.loco:
                still_samples.append(collector.loco[-1])
        assert still_samples
        still_dx = max(item["pose"]["x"] for item in still_samples) - min(
            item["pose"]["x"] for item in still_samples
        )
        assert still_dx < 0.03, still_dx
        assert all(item["balance"]["state"] == "stable" for item in still_samples[-10:]), still_samples[-10:]
        print(
            "P2 STANDING PASS "
            + json.dumps(
                {
                    "pelvis_height_m": stable["body_height"],
                    "torso_up_dot": stable["balance"]["torso_up_dot"],
                    "contact_forces_n": forces,
                    "idle_drift_m": round(still_dx, 6),
                },
                ensure_ascii=False,
            )
        )

        assert collector.imu and collector.joints, {
            "imu": len(collector.imu),
            "joints": len(collector.joints),
        }
        imu = collector.imu[-1]
        joints = collector.joints[-1]
        assert imu["simulation_backend"] == BACKEND and len(imu["quaternion"]) == 4, imu
        valid_joints = [joint for joint in joints["joints"] if joint.get("valid")]
        assert len(valid_joints) >= 20, len(valid_joints)
        assert any(abs(joint["q"]) > 0.2 for joint in valid_joints), valid_joints
        assert any(abs(joint["tau"]) > 0.1 for joint in valid_joints), valid_joints
        print(f"P2 PHYSICAL JOINT+IMU PASS valid_joints={len(valid_joints)}")

        wave = call("gesture", {"action": "wave", "duration": 5.0})
        assert wave["control_mode"] == "mujoco_joint_position_servo", wave
        assert wave["motion_semantics"] == "raise_left_arm_wave_wrist_then_lower", wave
        tracked = {
            "left_shoulder_pitch_joint": [],
            "left_shoulder_roll_joint": [],
            "left_wrist_yaw_joint": [],
            "right_shoulder_roll_joint": [],
            "waist_yaw_joint": [],
        }
        phases = set()
        wave_balance = []
        wave_deadline = time.monotonic() + 5.8
        while time.monotonic() < wave_deadline:
            rclpy.spin_once(collector, timeout_sec=0.1)
            if collector.joints:
                latest = {joint["name"]: joint for joint in collector.joints[-1]["joints"]}
                for name in tracked:
                    tracked[name].append(latest[name]["q"])
            if collector.loco:
                phases.add(collector.loco[-1]["gesture_phase"])
                wave_balance.append(collector.loco[-1]["balance"]["state"])
        assert tracked["left_shoulder_roll_joint"]
        assert max(tracked["left_shoulder_roll_joint"]) > 1.2, tracked
        wrist_range = max(tracked["left_wrist_yaw_joint"]) - min(
            tracked["left_wrist_yaw_joint"]
        )
        assert wrist_range > 0.9, wrist_range
        right_arm_range = max(tracked["right_shoulder_roll_joint"]) - min(
            tracked["right_shoulder_roll_joint"]
        )
        waist_range = max(tracked["waist_yaw_joint"]) - min(tracked["waist_yaw_joint"])
        assert right_arm_range < 0.15, right_arm_range
        assert waist_range < 0.15, waist_range
        assert {"raising", "waving", "lowering"} <= phases, phases
        assert wave_balance and all(state == "stable" for state in wave_balance), wave_balance
        gesture_info = call("gesture", {"action": "info"})
        assert gesture_info["gesture"] == "idle" and gesture_info["gesture_phase"] == "idle", gesture_info
        assert abs(tracked["left_shoulder_pitch_joint"][-1] - 0.2) < 0.15, tracked
        assert abs(tracked["left_shoulder_roll_joint"][-1]) < 0.15, tracked
        assert abs(tracked["left_wrist_yaw_joint"][-1]) < 0.15, tracked
        print(
            "P2 SEMANTIC WAVE PASS "
            f"left_shoulder_roll_max={max(tracked['left_shoulder_roll_joint']):.4f}rad "
            f"left_wrist_yaw_range={wrist_range:.4f}rad "
            f"right_arm_range={right_arm_range:.4f}rad waist_range={waist_range:.4f}rad"
        )

        call("sim_control", {"action": "reset", "seed": 23})
        collector.spin_until(lambda item: item["balance"]["state"] == "stable", 5.0, "pre-fall stable state")
        assist = call("sim_control", {"action": "set_balance_assist", "enabled": False})
        assert assist["balance_assist"] is False, assist
        push = call("sim_control", {"action": "push", "fx": 400.0, "fy": 0.0, "duration": 0.3})
        assert push["force_n"] == [400.0, 0.0], push
        fallen = collector.spin_until(
            lambda item: item["balance"]["state"] == "fallen",
            6.0,
            "fallen state after disabling assist and applying push",
        )
        assert fallen["physical_telemetry"]["balance_assist"] is False, fallen
        print(
            "P2 FALL DETECTION PASS "
            + json.dumps(
                {
                    "pelvis_height_m": fallen["balance"]["pelvis_height_m"],
                    "torso_up_dot": fallen["balance"]["torso_up_dot"],
                }
            )
        )

        recovered_reset = call("sim_control", {"action": "reset", "seed": 23})
        assert recovered_reset["pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}, recovered_reset
        recovered = collector.spin_until(
            lambda item: item["balance"]["state"] == "stable"
            and item["physical_telemetry"]["balance_assist"] is True,
            5.0,
            "stable state after reset",
        )
        assert recovered["balance"]["fallen"] is False, recovered
        print("P2 RESET RECOVERY PASS")
    finally:
        collector.destroy_node()
        rclpy.shutdown()

    call("gesture", {"action": "stop"})
    call("sim_control", {"action": "stop"})
    for sensor in TOPICS:
        assert call(sensor, {"action": "stop"})["state"] == "idle"
    print("P2 MUJOCO G1 ACCEPTANCE PASS")


if __name__ == "__main__":
    main()
