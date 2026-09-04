#!/usr/bin/env python3
"""G1 simulation Driver with interchangeable protocol and MuJoCo backends."""

from __future__ import annotations

from array import array
from io import BytesIO
import json
import math
import os
from pathlib import Path
import ssl
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib import request as urllib_request
import xml.etree.ElementTree as ET

from common import logsafe

logsafe.install()

import rclpy
from audio_msgs.msg import AudioChunk
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from PIL import Image, ImageDraw

from state import FAULT_MODES, SimulationState


MCP_PORT = int(os.environ.get("MCP_PORT", "15730"))
NAMESPACE = os.environ.get("SIM_NAMESPACE", "phanthymotus_sim_g1").strip("/")
AGENT_CORE_URL = os.environ.get("AGENT_CORE_URL", "https://agent-core:15678").rstrip("/")
MCP_ADVERTISE_URL = os.environ.get("MCP_ADVERTISE_URL", f"http://sim-driver:{MCP_PORT}/mcp")
URDF_PATH = Path(os.environ.get("SIM_G1_URDF", "/work/resource/g1_model.urdf"))
SIMULATION_BACKEND = os.environ.get("SIMULATION_BACKEND", "protocol").strip().lower()
MUJOCO_MODEL_PATH = Path(
    os.environ.get("SIM_MUJOCO_MODEL", "/work/resource/mujoco/g1/scene_29dof.xml")
)
DISPLAY_NAME = os.environ.get(
    "SIM_DRIVER_DISPLAY_NAME",
    "Simulated G1 (MuJoCo)" if SIMULATION_BACKEND == "mujoco" else "Simulated G1 (Protocol)",
)

LOW_LATENCY_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

TOPICS = {
    "mic": f"/{NAMESPACE}/mic/audio",
    "camera_rgb": f"/{NAMESPACE}/camera/rgb",
    "imu": f"/{NAMESPACE}/state/imu",
    "battery": f"/{NAMESPACE}/state/battery",
    "joints": f"/{NAMESPACE}/state/joints",
    "loco_state": f"/{NAMESPACE}/loco/state",
}

FORMATS = {
    "mic": "audio/pcm-16k",
    "camera_rgb": "image/jpeg",
    "imu": "data/json",
    "battery": "data/json",
    "joints": "sensor/skeleton",
    "loco_state": "data/json",
}


def _load_model(path: Path) -> tuple[str, list[str]]:
    if not path.is_file():
        raise FileNotFoundError(f"G1 URDF not found: {path}")
    urdf = path.read_text(encoding="utf-8")
    root = ET.fromstring(urdf)
    names = [
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib.get("type") != "fixed" and joint.attrib.get("name")
    ]
    if not names:
        raise ValueError("G1 URDF contains no movable joints")
    return urdf, names


class SimPublisherNode(Node):
    def __init__(self, state, joint_names: list[str]):
        super().__init__("phanthymotus_sim_g1")
        self.state = state
        self.joint_names = joint_names
        self._active: set[str] = set()
        self._active_lock = threading.RLock()
        self._audio_phase = 0.0
        self._metrics = {name: 0 for name in TOPICS}

        self._mic_pub = self.create_publisher(AudioChunk, TOPICS["mic"], LOW_LATENCY_QOS)
        self._camera_pub = self.create_publisher(CompressedImage, TOPICS["camera_rgb"], LOW_LATENCY_QOS)
        self._imu_pub = self.create_publisher(String, TOPICS["imu"], LOW_LATENCY_QOS)
        self._battery_pub = self.create_publisher(String, TOPICS["battery"], LOW_LATENCY_QOS)
        self._joints_pub = self.create_publisher(String, TOPICS["joints"], LOW_LATENCY_QOS)
        self._loco_pub = self.create_publisher(String, TOPICS["loco_state"], LOW_LATENCY_QOS)

        self.create_timer(0.05, self._publish_fast_state)
        self.create_timer(1.0, self._publish_battery)
        self.create_timer(0.2, self._publish_camera)
        self.create_timer(0.032, self._publish_audio)

    def set_active(self, name: str, active: bool) -> None:
        with self._active_lock:
            if active:
                self._active.add(name)
            else:
                self._active.discard(name)

    def is_active(self, name: str) -> bool:
        with self._active_lock:
            return name in self._active

    def metrics(self, name: str) -> dict:
        return {
            "active": self.is_active(name),
            "published": self._metrics.get(name, 0),
        }

    @staticmethod
    def _json_message(payload: dict) -> String:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        return message

    def _publish_fast_state(self) -> None:
        self.state.step()
        if self.is_active("loco_state"):
            self._loco_pub.publish(self._json_message(self.state.loco_snapshot()))
            self._metrics["loco_state"] += 1
        if self.is_active("imu"):
            self._imu_pub.publish(self._json_message(self.state.imu_snapshot()))
            self._metrics["imu"] += 1
        if self.is_active("joints"):
            self._joints_pub.publish(self._json_message(self.state.joints_snapshot(self.joint_names)))
            self._metrics["joints"] += 1

    def _publish_battery(self) -> None:
        if not self.is_active("battery"):
            return
        self._battery_pub.publish(self._json_message(self.state.battery_snapshot()))
        self._metrics["battery"] += 1

    def _publish_camera(self) -> None:
        if not self.is_active("camera_rgb") or self.state.snapshot()["fault_mode"] == "drop_camera":
            return
        snapshot = self.state.snapshot()
        image = Image.new("RGB", (640, 360), (20, 27, 38))
        draw = ImageDraw.Draw(image)
        for x in range(0, 641, 40):
            draw.line((x, 0, x, 360), fill=(35, 46, 62), width=1)
        for y in range(0, 361, 40):
            draw.line((0, y, 640, y), fill=(35, 46, 62), width=1)

        pose = snapshot["pose"]
        cx = int(320 + pose["x"] * 45.0)
        cy = int(180 - pose["y"] * 45.0)
        yaw = pose["yaw"]
        tip = (cx + int(28 * math.cos(yaw)), cy - int(28 * math.sin(yaw)))
        draw.ellipse((cx - 12, cy - 12, cx + 12, cy + 12), fill=(0, 187, 255), outline=(255, 255, 255), width=2)
        draw.line((cx, cy, tip[0], tip[1]), fill=(255, 196, 64), width=5)
        backend_label = (
            "MUJOCO PHYSICS" if snapshot["simulation_backend"].startswith("mujoco_") else "PROTOCOL FIXTURE"
        )
        draw.text((18, 16), f"SIMULATED G1 - {backend_label}", fill=(245, 248, 252))
        draw.text((18, 42), f"x={pose['x']:.2f}  y={pose['y']:.2f}  yaw={yaw:.2f}", fill=(139, 213, 255))
        draw.text((18, 66), f"seq={snapshot['sequence']}  fault={snapshot['fault_mode']}", fill=(172, 180, 192))

        encoded = BytesIO()
        image.save(encoded, format="JPEG", quality=82, optimize=False)
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "sim_camera"
        message.format = "jpeg"
        message.data = list(encoded.getvalue())
        self._camera_pub.publish(message)
        self._metrics["camera_rgb"] += 1

    def _publish_audio(self) -> None:
        if not self.is_active("mic") or self.state.snapshot()["fault_mode"] == "drop_audio":
            return
        sample_count = 512
        sample_rate = 16000
        frequency = 440.0
        samples = array("h")
        for index in range(sample_count):
            sample = int(4200 * math.sin(self._audio_phase + 2.0 * math.pi * frequency * index / sample_rate))
            samples.append(sample)
        self._audio_phase = (self._audio_phase + 2.0 * math.pi * frequency * sample_count / sample_rate) % (2.0 * math.pi)
        if sys.byteorder != "little":
            samples.byteswap()
        payload = samples.tobytes()
        message = AudioChunk()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "sim_mic"
        message.format = "audio/pcm-16k"
        message.data = list(payload)
        self._mic_pub.publish(message)
        self._metrics["mic"] += 1


class SimG1Bundle:
    SENSOR_NAMES = tuple(TOPICS)

    def __init__(self, node: SimPublisherNode, state, urdf: str):
        self.node = node
        self.state = state
        self.urdf = urdf
        self.backend_name = getattr(state, "backend_name", "protocol_only_no_physics")
        self.physics_enabled = self.backend_name.startswith("mujoco_")
        self._loco_ready = False
        self._gesture_ready = False
        self._control_ready = False
        self._tools = self._build_tools()

    @staticmethod
    def _sensor_tool(name: str, description: str) -> dict:
        return {
            "name": name,
            "type": "sensor",
            "multiInstance": False,
            "description": f"SIMULATION ONLY — {description}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": TOPICS[name], "format": FORMATS[name]}],
        }

    def _build_tools(self) -> list[dict]:
        if self.physics_enabled:
            imu_description = "MuJoCo pelvis orientation, angular velocity and acceleration"
            joints_description = "MuJoCo G1 joint position, velocity and applied torque"
            loco_state_description = "MuJoCo pose, contact force and explicit balance diagnostics at 20 Hz"
        else:
            imu_description = "deterministic orientation, gyro and acceleration JSON"
            joints_description = "G1-named joint fixture for the skeleton renderer"
            loco_state_description = "integrated protocol pose and velocity state at 20 Hz"
            loco_description = "SIMULATION ONLY — protocol-level velocity control; no dynamics or balance guarantee"
            move_description = "Integrate a bounded protocol-level velocity command"
            velocity_limit = "[-1, 1]"
            yaw_limit = "[-2, 2]"

        control_actions = ["reset", "pause", "resume", "set_fault"]
        control_properties = {
            "action": {
                "type": "string",
                "enum": control_actions,
                "description": "Simulation control action",
            },
            "seed": {"type": "integer", "description": "Deterministic fixture seed"},
            "fault_mode": {"type": "string", "enum": list(FAULT_MODES), "description": "Injected fault"},
        }
        control_params = {
            "reset": {"params": ["seed"], "description": "Reset pose, motion, time and faults"},
            "pause": {"params": [], "description": "Pause state integration"},
            "resume": {"params": [], "description": "Resume state integration"},
            "set_fault": {"params": ["fault_mode"], "description": "Inject or clear one bounded fault"},
        }
        if self.physics_enabled:
            control_actions.extend(["set_balance_assist", "push"])
            control_properties.update(
                {
                    "enabled": {"type": "boolean", "description": "Enable virtual base pose servo"},
                    "fx": {"type": "number", "description": "World X push force N [-500, 500]"},
                    "fy": {"type": "number", "description": "World Y push force N [-500, 500]"},
                    "duration": {"type": "number", "description": "Push duration seconds [0.02, 1.0]"},
                }
            )
            control_params.update(
                {
                    "set_balance_assist": {
                        "params": ["enabled"],
                        "description": "Enable or disable the explicitly reported virtual stabilization assist",
                    },
                    "push": {
                        "params": ["fx", "fy", "duration"],
                        "description": "Apply a bounded external planar force for fall/recovery tests",
                    },
                }
            )

        tools = [
            self._sensor_tool("mic", "deterministic PCM_S16_LE 16000 Hz mono tone fixture"),
            self._sensor_tool("camera_rgb", "deterministic 640x360 JPEG scene with live backend pose"),
            self._sensor_tool("imu", imu_description),
            self._sensor_tool("battery", "deterministic BMS telemetry JSON"),
            self._sensor_tool("joints", joints_description),
            self._sensor_tool("loco_state", loco_state_description),
            {
                "name": "model",
                "type": "resource",
                "multiInstance": False,
                "description": "SIMULATION ONLY — G1 URDF used by the WebUI skeleton renderer",
                "inputSchema": {"type": "object", "properties": {}},
            },
        ]
        if not self.physics_enabled:
            tools.append({
                "name": "loco",
                "type": "actuator",
                "multiInstance": False,
                "description": loco_description,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["move", "stop_move"],
                            "description": "Action to perform",
                        },
                        "vx": {"type": "number", "description": f"Forward reference velocity m/s {velocity_limit}"},
                        "vy": {"type": "number", "description": f"Lateral reference velocity m/s {velocity_limit}"},
                        "vyaw": {"type": "number", "description": f"Yaw reference velocity rad/s {yaw_limit}"},
                        "duration": {"type": "number", "description": "Seconds; <=0 runs until stop_move"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "move": {
                            "params": ["vx", "vy", "vyaw", "duration"],
                            "description": move_description,
                        },
                        "stop_move": {"params": [], "description": "Stop simulated movement immediately"},
                    },
                },
            })
        if self.physics_enabled:
            tools.append(
                {
                    "name": "gesture",
                    "type": "actuator",
                    "multiInstance": False,
                    "description": (
                        "SIMULATION ONLY — semantic left-arm wave: raise the arm, "
                        "oscillate the wrist, then return to standing; not locomotion"
                    ),
                    "inputSchema": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["wave", "stop_wave"],
                                "description": "Gesture action",
                            },
                            "duration": {
                                "type": "number",
                                "description": "Wave duration seconds [4, 10]",
                            },
                        },
                        "required": ["action"],
                        "x-action-params": {
                            "wave": {
                                "params": ["duration"],
                                "description": "Raise left arm, wave at the wrist, then lower it smoothly",
                            },
                            "stop_wave": {
                                "params": [],
                                "description": "Return the arm controller to the standing posture",
                            },
                        },
                    },
                }
            )
        tools.append(
            {
                "name": "sim_control",
                "type": "actuator",
                "multiInstance": False,
                "description": "SIMULATION ONLY — deterministic reset, pause and fault injection controls",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        **control_properties,
                    },
                    "required": ["action"],
                    "x-action-params": control_params,
                },
            }
        )
        return tools

    def get_tools(self) -> list[dict]:
        return self._tools

    def status(self) -> dict:
        return {
            "ok": True,
            "name": "sim-g1-device-bundle",
            "simulation": True,
            "simulation_backend": self.backend_name,
            "namespace": NAMESPACE,
            "state": self.state.snapshot(),
            "sensors": {name: self.node.metrics(name) for name in self.SENSOR_NAMES},
        }

    def dispatch(self, name: str, arguments: dict) -> dict | None:
        args = dict(arguments)
        action = args.pop("action", name)
        args["_tool_name"] = name

        if name in self.SENSOR_NAMES:
            if action == "start":
                self.node.set_active(name, True)
            elif action == "stop":
                self.node.set_active(name, False)
            elif action != "info":
                return None
            metrics = self.node.metrics(name)
            return {
                "state": "running" if metrics["active"] else "idle",
                "simulation": True,
                "simulation_backend": self.backend_name,
                "published": metrics["published"],
                "topic_out": [{"topic": TOPICS[name], "format": FORMATS[name]}],
            }

        if name == "model":
            if action == "model":
                return {"urdf": self.urdf, "simulation": True, "simulation_backend": self.backend_name}
            if action == "info":
                return {"state": "ready", "simulation": True}
            if action == "start":
                return {"state": "ready", "simulation": True}
            if action == "stop":
                return {"state": "idle", "simulation": True}
            return None

        if name == "loco":
            if self.physics_enabled:
                raise ValueError(
                    "loco is unavailable in MuJoCo P2: the official 29DoF "
                    "locomotion policy has not passed sim2sim acceptance"
                )
            if action == "start":
                self._loco_ready = True
                return {"state": "ready", "simulation": True}
            if action == "stop":
                self._loco_ready = False
                return {**self.state.stop_move(), "state": "idle", "simulation": True}
            if action == "info":
                return {"state": "ready" if self._loco_ready else "idle", "simulation": True}
            if not self._loco_ready:
                raise ValueError("loco must be started before motion commands")
            if action == "move":
                return {
                    **self.state.command_move(
                        args.get("vx", 0.0),
                        args.get("vy", 0.0),
                        args.get("vyaw", 0.0),
                        args.get("duration", 0.0),
                    ),
                    "simulation": True,
                }
            if action == "stop_move":
                return {**self.state.stop_move(), "simulation": True}
            return None

        if name == "gesture" and self.physics_enabled:
            if action == "start":
                self._gesture_ready = True
                return {"state": "ready", "simulation": True}
            if action == "stop":
                self._gesture_ready = False
                return {**self.state.stop_gesture(), "simulation": True}
            if action == "info":
                return {
                    "state": "ready" if self._gesture_ready else "idle",
                    "simulation": True,
                    "simulation_backend": self.backend_name,
                    "gesture": self.state.snapshot()["gesture"],
                    "gesture_phase": self.state.snapshot()["gesture_phase"],
                }
            if not self._gesture_ready:
                raise ValueError("gesture must be started before gesture commands")
            if action == "wave":
                return {
                    **self.state.command_wave(args.get("duration", 4.0)),
                    "simulation": True,
                }
            if action == "stop_wave":
                return {**self.state.stop_gesture(), "simulation": True}
            return None

        if name == "sim_control":
            if action == "start":
                self._control_ready = True
                return {"state": "ready", "simulation": True}
            if action == "stop":
                self._control_ready = False
                return {"state": "idle", "simulation": True}
            if action == "info":
                return {"state": "ready" if self._control_ready else "idle", **self.status()}
            if not self._control_ready:
                raise ValueError("sim_control must be started before control actions")
            if action == "reset":
                seed = args.get("seed")
                return self.state.reset(seed=int(seed) if seed is not None else None)
            if action == "pause":
                return {**self.state.set_paused(True), "simulation": True}
            if action == "resume":
                return {**self.state.set_paused(False), "simulation": True}
            if action == "set_fault":
                return {**self.state.set_fault(str(args.get("fault_mode", "none"))), "simulation": True}
            if action == "set_balance_assist" and self.physics_enabled:
                return {**self.state.set_balance_assist(bool(args.get("enabled", True))), "simulation": True}
            if action == "push" and self.physics_enabled:
                return {
                    **self.state.apply_push(
                        args.get("fx", 0.0),
                        args.get("fy", 0.0),
                        args.get("duration", 0.2),
                    ),
                    "simulation": True,
                }
            return None

        return None


bundle: SimG1Bundle | None = None


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
    handler.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
    handler.end_headers()
    handler.wfile.write(body)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        message = (fmt % args).encode("unicode_escape").decode("ascii")[:240]
        if '"POST /mcp' not in message or " 200 " not in message:
            print(f"[mcp] {self.address_string()} {message}", flush=True)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
        self.end_headers()

    def do_GET(self):
        if self.path == "/health" and bundle is not None:
            _json_response(self, 200, bundle.status())
        else:
            _json_response(self, 404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path != "/mcp" or bundle is None:
            _json_response(self, 404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            rpc = json.loads(self.rfile.read(length))
        except Exception:
            _json_response(self, 400, {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}})
            return

        request_id = rpc.get("id")
        if request_id is None:
            self.send_response(202)
            self.end_headers()
            return

        method = rpc.get("method", "")
        params = rpc.get("params") or {}
        try:
            if method == "initialize":
                result = {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "sim-g1-device-bundle", "version": "1.0.0"},
                }
            elif method == "tools/list":
                result = {"tools": bundle.get_tools()}
            elif method == "tools/call":
                name = params.get("name", "")
                dispatched = bundle.dispatch(name, params.get("arguments") or {})
                if dispatched is None:
                    raise KeyError(f"unknown tool or action: {name}")
                result = {"content": [{"type": "text", "text": json.dumps(dispatched, ensure_ascii=False)}]}
            else:
                raise KeyError(f"method not found: {method}")
            _json_response(self, 200, {"jsonrpc": "2.0", "id": request_id, "result": result})
        except ValueError as exc:
            _json_response(self, 200, {"jsonrpc": "2.0", "id": request_id, "error": {"code": -32602, "message": str(exc)}})
        except KeyError as exc:
            _json_response(self, 200, {"jsonrpc": "2.0", "id": request_id, "error": {"code": -32601, "message": str(exc)}})
        except Exception as exc:
            _json_response(self, 200, {"jsonrpc": "2.0", "id": request_id, "error": {"code": -32603, "message": str(exc)}})


def _start_registration() -> None:
    payload = json.dumps(
        {
            "name": DISPLAY_NAME,
            "transport": "http",
            "url": MCP_ADVERTISE_URL,
            "category": "driver",
        }
    ).encode("utf-8")
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE

    def run():
        while rclpy.ok():
            try:
                req = urllib_request.Request(
                    f"{AGENT_CORE_URL}/api/mcp",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib_request.urlopen(req, timeout=5, context=context) as response:
                    if response.status != 200:
                        raise RuntimeError(f"registration returned HTTP {response.status}")
                print(f"[register] heartbeat ok -> {AGENT_CORE_URL}", flush=True)
                time.sleep(30)
            except Exception as exc:
                print(f"[register] failed: {exc}; retrying in 5s", flush=True)
                time.sleep(5)

    threading.Thread(target=run, daemon=True, name="register").start()


def main() -> None:
    global bundle
    urdf, joint_names = _load_model(URDF_PATH)
    rclpy.init()
    if SIMULATION_BACKEND == "protocol":
        state = SimulationState(seed=int(os.environ.get("SIM_SEED", "7")))
    elif SIMULATION_BACKEND == "mujoco":
        from mujoco_backend import MujocoSimulationState

        state = MujocoSimulationState(
            MUJOCO_MODEL_PATH,
            seed=int(os.environ.get("SIM_SEED", "7")),
        )
    else:
        raise ValueError(f"SIMULATION_BACKEND must be protocol or mujoco, got {SIMULATION_BACKEND!r}")
    node = SimPublisherNode(state, joint_names)
    bundle = SimG1Bundle(node, state, urdf)

    server = ThreadingHTTPServer(("0.0.0.0", MCP_PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True, name="mcp-http").start()
    _start_registration()

    print(f"[sim-g1] MCP -> http://0.0.0.0:{MCP_PORT}/mcp", flush=True)
    print(
        f"[sim-g1] namespace=/{NAMESPACE} joints={len(joint_names)} "
        f"seed={state.snapshot()['seed']} backend={getattr(state, 'backend_name', 'protocol_only_no_physics')}",
        flush=True,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        state.stop_move()
        if hasattr(state, "stop_gesture"):
            state.stop_gesture()
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
