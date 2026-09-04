#!/usr/bin/env python3
"""Cross-container MCP and ROS acceptance for the protocol-level G1 simulator."""

from __future__ import annotations

import json
import ssl
import time
from urllib import request as urllib_request

import rclpy
from audio_msgs.msg import AudioChunk
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


MCP_URL = "http://sim-driver:15730/mcp"
CORE_URL = "https://127.0.0.1:15678"
EXPECTED_TOOLS = {
    "mic",
    "camera_rgb",
    "imu",
    "battery",
    "joints",
    "model",
    "loco_state",
    "loco",
    "sim_control",
}
TOPICS = {
    "mic": "/phanthymotus_sim_g1/mic/audio",
    "camera_rgb": "/phanthymotus_sim_g1/camera/rgb",
    "imu": "/phanthymotus_sim_g1/state/imu",
    "battery": "/phanthymotus_sim_g1/state/battery",
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
    text = response["result"]["content"][0]["text"]
    return json.loads(text)


def call_expect_error(name: str, arguments: dict, code: int) -> dict:
    response = rpc("tools/call", {"name": name, "arguments": arguments})
    assert response.get("error", {}).get("code") == code, response
    return response["error"]


def core_registry() -> list[dict]:
    context = ssl._create_unverified_context()
    with urllib_request.urlopen(f"{CORE_URL}/api/mcp", timeout=5, context=context) as response:
        payload = json.loads(response.read())
    assert payload["code"] == 200, payload
    return payload["data"]


def core_call(mcp_id: str, name: str, arguments: dict) -> dict:
    context = ssl._create_unverified_context()
    payload = json.dumps({"tool": name, "arguments": arguments}).encode()
    req = urllib_request.Request(
        f"{CORE_URL}/api/mcp/{mcp_id}/call",
        data=payload,
        headers={"Content-Type": "application/json"},
    )
    with urllib_request.urlopen(req, timeout=10, context=context) as response:
        result = json.loads(response.read())
    assert result["code"] == 200, result
    content = result["data"]
    assert isinstance(content, list) and content and content[0]["type"] == "text", result
    return json.loads(content[0]["text"])


class Collector(Node):
    def __init__(self):
        super().__init__("sim_g1_p1_acceptance")
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.audio = []
        self.camera = []
        self.imu = []
        self.battery = []
        self.joints = []
        self.loco = []
        self.create_subscription(AudioChunk, TOPICS["mic"], self.audio.append, qos)
        self.create_subscription(CompressedImage, TOPICS["camera_rgb"], self.camera.append, qos)
        self.create_subscription(String, TOPICS["imu"], lambda msg: self.imu.append(json.loads(msg.data)), qos)
        self.create_subscription(String, TOPICS["battery"], lambda msg: self.battery.append(json.loads(msg.data)), qos)
        self.create_subscription(String, TOPICS["joints"], lambda msg: self.joints.append(json.loads(msg.data)), qos)
        self.create_subscription(String, TOPICS["loco_state"], lambda msg: self.loco.append(json.loads(msg.data)), qos)

    def complete(self) -> bool:
        return all((self.audio, self.camera, self.imu, self.battery, self.joints, self.loco))


def main() -> None:
    init = rpc("initialize")
    assert init["result"]["serverInfo"]["name"] == "sim-g1-device-bundle", init
    tools = rpc("tools/list")["result"]["tools"]
    tool_names = {tool["name"] for tool in tools}
    assert tool_names == EXPECTED_TOOLS, tool_names
    assert all("SIMULATION ONLY" in tool["description"] for tool in tools), tools
    print(f"MCP CONTRACT PASS tools={sorted(tool_names)}")

    registry_entry = next(
        item for item in core_registry() if item.get("server_name") == "sim-g1-device-bundle"
    )
    assert registry_entry["name"] == "Simulated G1 (Protocol)", registry_entry
    assert registry_entry["url"] == MCP_URL, registry_entry
    assert {tool["name"] for tool in registry_entry["tools"]} == EXPECTED_TOOLS, registry_entry
    print(f"CORE REGISTRATION PASS id={registry_entry['id']}")

    core_mcp_id = registry_entry["id"]
    core_camera = core_call(core_mcp_id, "camera_rgb", {"action": "start"})
    assert core_camera["state"] == "running" and core_camera["simulation"] is True, core_camera
    assert core_call(core_mcp_id, "loco", {"action": "start"})["state"] == "ready"
    core_move = core_call(
        core_mcp_id,
        "loco",
        {"action": "move", "vx": 0.1, "vy": 0.0, "vyaw": 0.0, "duration": 0.2},
    )
    assert core_move["state"] == "moving" and core_move["simulation"] is True, core_move
    assert core_call(core_mcp_id, "loco", {"action": "stop"})["state"] == "idle"
    assert core_call(core_mcp_id, "camera_rgb", {"action": "stop"})["state"] == "idle"
    print("CORE INTERMEDIARY CALL PASS tools=camera_rgb,loco")

    model = call("model", {})
    assert model["simulation"] is True and '<robot name="g1_' in model["urdf"], model.keys()
    print(f"MODEL PASS urdf_bytes={len(model['urdf'].encode())}")

    error = call_expect_error(
        "loco",
        {"action": "move", "vx": 0.2, "vy": 0.0, "vyaw": 0.0},
        -32602,
    )
    assert "must be started" in error["message"], error
    print("LIFECYCLE REJECTION PASS")

    for sensor in TOPICS:
        result = call(sensor, {"action": "start"})
        assert result["state"] == "running", (sensor, result)
        assert result["topic_out"][0]["topic"] == TOPICS[sensor], (sensor, result)
    assert call("loco", {"action": "start"})["state"] == "ready"
    assert call("sim_control", {"action": "start"})["state"] == "ready"
    call("sim_control", {"action": "reset", "seed": 23})

    rclpy.init()
    collector = Collector()
    try:
        move = call(
            "loco",
            {"action": "move", "vx": 0.4, "vy": 0.0, "vyaw": 0.25, "duration": 1.2},
        )
        assert move["state"] == "moving" and move["simulation"] is True, move
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.1)
            if collector.complete() and len(collector.loco) >= 8:
                break
        assert collector.complete(), {
            "audio": len(collector.audio),
            "camera": len(collector.camera),
            "imu": len(collector.imu),
            "battery": len(collector.battery),
            "joints": len(collector.joints),
            "loco": len(collector.loco),
        }
        assert len(collector.audio[-1].data) >= 1024
        assert any(collector.audio[-1].data)
        assert collector.audio[-1].format == "audio/pcm-16k"
        camera_bytes = bytes(collector.camera[-1].data)
        assert camera_bytes.startswith(b"\xff\xd8") and camera_bytes.endswith(b"\xff\xd9")
        assert collector.imu[-1]["simulation"] is True
        assert collector.battery[-1]["soc_percent"] > 0
        assert len(collector.joints[-1]["joints"]) >= 20
        assert collector.joints[-1]["joints"][0]["name"].endswith("_joint")
        assert max(item["pose"]["x"] for item in collector.loco) > 0.08, collector.loco
        assert all(item["robot_morphology"] == "humanoid_biped" for item in collector.loco)
        assert all(item["simulation_backend"] == "protocol_only_no_physics" for item in collector.loco)
        assert all(item["physical_telemetry"]["valid"] is False for item in collector.loco)
        assert all(item["foot_force"] is None and item["foot_force_valid"] is False for item in collector.loco)
        print(
            "ROS DATA PASS "
            + json.dumps(
                {
                    "audio": len(collector.audio),
                    "camera": len(collector.camera),
                    "imu": len(collector.imu),
                    "battery": len(collector.battery),
                    "joints": len(collector.joints),
                    "loco": len(collector.loco),
                    "max_x": max(item["pose"]["x"] for item in collector.loco),
                    "jpeg_bytes": len(camera_bytes),
                },
                ensure_ascii=False,
            )
        )
    finally:
        collector.destroy_node()
        rclpy.shutdown()

    camera_before = call("camera_rgb", {"action": "info"})["published"]
    call("sim_control", {"action": "set_fault", "fault_mode": "drop_camera"})
    time.sleep(0.8)
    camera_during = call("camera_rgb", {"action": "info"})["published"]
    assert camera_during - camera_before <= 1, (camera_before, camera_during)
    call("sim_control", {"action": "set_fault", "fault_mode": "none"})
    time.sleep(0.6)
    camera_after = call("camera_rgb", {"action": "info"})["published"]
    assert camera_after > camera_during, (camera_during, camera_after)
    print("FAULT INJECTION PASS mode=drop_camera")

    call("loco", {"action": "stop_move"})
    call("loco", {"action": "stop"})
    reset = call("sim_control", {"action": "reset", "seed": 7})
    assert reset["pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}, reset
    assert reset["velocity"] == {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}, reset
    call("sim_control", {"action": "stop"})
    for sensor in TOPICS:
        assert call(sensor, {"action": "stop"})["state"] == "idle"
    print("P1 PROTOCOL G1 ACCEPTANCE PASS")


if __name__ == "__main__":
    main()
