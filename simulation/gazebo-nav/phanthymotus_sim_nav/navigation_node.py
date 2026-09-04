from __future__ import annotations

import json
import math
import os
import ssl
import struct
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib import request as urllib_request

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from phanthymotus_sim_nav.goal_result import terminal_goal_update

MCP_PORT = int(os.environ.get("MCP_PORT", "15731"))
AGENT_CORE_URL = os.environ.get("AGENT_CORE_URL", "https://agent-core:15678").rstrip("/")
MCP_ADVERTISE_URL = os.environ.get("MCP_ADVERTISE_URL", f"http://gazebo-nav:{MCP_PORT}/mcp")
ROOT = "/phanthymotus_sim_nav"
LOCALIZATION_MODE = os.environ.get("LOCALIZATION_MODE", "ground_truth")
if LOCALIZATION_MODE not in {"ground_truth", "amcl"}:
    raise RuntimeError(f"unsupported LOCALIZATION_MODE: {LOCALIZATION_MODE}")
ODOMETRY_MODE = os.environ.get("ODOMETRY_MODE", "ideal")
if ODOMETRY_MODE not in {"ideal", "deterministic_scale"}:
    raise RuntimeError(f"unsupported ODOMETRY_MODE: {ODOMETRY_MODE}")


def positive_float_env(name, default):
    value = float(os.environ.get(name, default))
    if not math.isfinite(value) or value <= 0:
        raise RuntimeError(f"{name} must be a positive finite number")
    return value


ODOMETRY_LINEAR_SCALE = positive_float_env("ODOMETRY_LINEAR_SCALE", "1.04")
ODOMETRY_YAW_SCALE = positive_float_env("ODOMETRY_YAW_SCALE", "1.03")
RELOCALIZATION_VARIANCE_MAX = positive_float_env("RELOCALIZATION_VARIANCE_MAX", "0.25")
RELOCALIZATION_MIN_SECONDS = positive_float_env("RELOCALIZATION_MIN_SECONDS", "2.0")
RELOCALIZATION_SCAN_SECONDS = positive_float_env("RELOCALIZATION_SCAN_SECONDS", "18.0")
RELOCALIZATION_SCAN_SPEED = positive_float_env("RELOCALIZATION_SCAN_SPEED", "0.35")


class NavigationNode(Node):
    def __init__(self):
        super().__init__("phanthymotus_sim_navigation")
        self._lock = threading.RLock()
        self._map = None
        self._pose = None
        self._ground_truth_pose = None
        self._odom_pose = None
        self._odom_origin = None
        self._raw_odom_last_yaw = None
        self._scaled_odom_yaw = None
        self._localization_covariance = None
        self._localization_ready = False
        self._relocalizing = False
        self._relocalization_started = None
        self._relocalization_stable_samples = 0
        self._relocalization_attempts = 0
        self._relocalization_last_error = ""
        self._relocalization_scan_active = False
        self._last_scan = self._last_odom = self._last_map = self._last_localization = self._last_ground_truth = None
        self._map_active = False
        self._nav2_lifecycle = {"id": 0, "label": "unknown"}
        self._amcl_lifecycle = {"id": 0, "label": "not_required" if LOCALIZATION_MODE == "ground_truth" else "unknown"}
        self._nav2_state_future = None
        self._amcl_state_future = None
        self._goal = {"state": "idle", "goal_id": None, "error": ""}
        self._goal_handle = None
        self._cancel_requested_goal_id = None
        self._scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._mapping_pub = self.create_publisher(UInt8MultiArray, f"{ROOT}/mapping", qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, f"{ROOT}/navigation/status", status_qos)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(LaserScan, f"{ROOT}/gz_scan_raw", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{ROOT}/gz_odom_raw", self._on_odom, 20)
        self.create_subscription(TFMessage, "/world/synthetic_room/dynamic_pose/info", self._on_ground_truth, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, status_qos)
        self._action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._nav2_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self._amcl_state_client = self.create_client(GetState, "/amcl/get_state")
        self._relocalize_client = self.create_client(Empty, "/reinitialize_global_localization")
        self.create_timer(0.5, self._poll_nav2_state)
        if LOCALIZATION_MODE == "amcl":
            self.create_timer(0.5, self._poll_amcl_state)
            self.create_timer(0.1, self._drive_relocalization_scan)
        self.create_timer(1.0, self._tick)
        stamp = self.get_clock().now().to_msg()
        base_to_scan = TransformStamped()
        base_to_scan.header.stamp = stamp
        base_to_scan.header.frame_id = "base_link"
        base_to_scan.child_frame_id = "base_scan"
        base_to_scan.transform.translation.x = 0.12
        base_to_scan.transform.translation.z = 0.18
        base_to_scan.transform.rotation.w = 1.0
        transforms = [base_to_scan]
        if LOCALIZATION_MODE == "ground_truth":
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = stamp
            map_to_odom.header.frame_id = "map"
            map_to_odom.child_frame_id = "odom"
            map_to_odom.transform.rotation.w = 1.0
            transforms.append(map_to_odom)
        self._static_tf.sendTransform(transforms)

    def _on_scan(self, msg):
        msg.header.frame_id = "base_scan"
        self._scan_pub.publish(msg)
        with self._lock: self._last_scan = time.monotonic()

    def _on_odom(self, msg):
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        pose = msg.pose.pose
        raw_pose = (pose.position.x, pose.position.y, self._yaw(pose.orientation))
        odom_pose = self._scaled_odom_pose(raw_pose)
        pose.position.x, pose.position.y = odom_pose[:2]
        pose.orientation.x = pose.orientation.y = 0.0
        pose.orientation.z = math.sin(odom_pose[2] / 2.0)
        pose.orientation.w = math.cos(odom_pose[2] / 2.0)
        if ODOMETRY_MODE == "deterministic_scale":
            msg.twist.twist.linear.x *= ODOMETRY_LINEAR_SCALE
            msg.twist.twist.angular.z *= ODOMETRY_YAW_SCALE
        self._odom_pub.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(t)
        with self._lock:
            self._last_odom = time.monotonic()
            self._odom_pose = odom_pose
            if LOCALIZATION_MODE == "ground_truth":
                self._pose = odom_pose
                self._last_localization = self._last_odom
                self._localization_ready = True

    def _scaled_odom_pose(self, raw_pose):
        if ODOMETRY_MODE == "ideal":
            return raw_pose
        x, y, yaw = raw_pose
        with self._lock:
            if self._odom_origin is None:
                self._odom_origin = raw_pose
                self._raw_odom_last_yaw = yaw
                self._scaled_odom_yaw = yaw
            else:
                delta = math.atan2(
                    math.sin(yaw - self._raw_odom_last_yaw),
                    math.cos(yaw - self._raw_odom_last_yaw),
                )
                self._scaled_odom_yaw += delta * ODOMETRY_YAW_SCALE
                self._raw_odom_last_yaw = yaw
            origin_x, origin_y, _ = self._odom_origin
            return (
                origin_x + (x - origin_x) * ODOMETRY_LINEAR_SCALE,
                origin_y + (y - origin_y) * ODOMETRY_LINEAR_SCALE,
                math.atan2(math.sin(self._scaled_odom_yaw), math.cos(self._scaled_odom_yaw)),
            )

    def _on_ground_truth(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != "planar_base":
                continue
            pose = transform.transform
            with self._lock:
                self._ground_truth_pose = (
                    pose.translation.x,
                    pose.translation.y,
                    self._yaw(pose.rotation),
                )
                self._last_ground_truth = time.monotonic()
            return

    def _on_amcl_pose(self, msg):
        if LOCALIZATION_MODE != "amcl":
            return
        pose = msg.pose.pose
        with self._lock:
            self._pose = (pose.position.x, pose.position.y, self._yaw(pose.orientation))
            self._localization_covariance = {
                "x": msg.pose.covariance[0],
                "y": msg.pose.covariance[7],
                "yaw": msg.pose.covariance[35],
            }
            self._last_localization = time.monotonic()
            if self._relocalizing:
                elapsed = self._last_localization - self._relocalization_started
                converged = elapsed >= max(RELOCALIZATION_MIN_SECONDS, RELOCALIZATION_SCAN_SECONDS) and all(
                    abs(value) <= RELOCALIZATION_VARIANCE_MAX
                    for value in self._localization_covariance.values()
                )
                self._relocalization_stable_samples = self._relocalization_stable_samples + 1 if converged else 0
                if self._relocalization_stable_samples >= 3:
                    self._relocalizing = False
                    self._relocalization_last_error = ""
            self._localization_ready = not self._relocalizing

    def _on_map(self, msg):
        with self._lock:
            self._map = msg
            self._last_map = time.monotonic()

    @staticmethod
    def _age(value):
        return None if value is None else round(time.monotonic() - value, 3)

    @staticmethod
    def _yaw(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _poll_nav2_state(self):
        if self._nav2_state_future is not None and not self._nav2_state_future.done():
            return
        if not self._nav2_state_client.service_is_ready():
            with self._lock:
                self._nav2_lifecycle = {"id": 0, "label": "unavailable"}
            return
        self._nav2_state_future = self._nav2_state_client.call_async(GetState.Request())
        self._nav2_state_future.add_done_callback(self._on_nav2_state)

    def _on_nav2_state(self, future):
        try:
            current = future.result().current_state
            lifecycle = {"id": int(current.id), "label": current.label}
        except Exception as exc:
            lifecycle = {"id": 0, "label": f"error: {exc}"}
        with self._lock:
            self._nav2_lifecycle = lifecycle

    def _poll_amcl_state(self):
        if self._amcl_state_future is not None and not self._amcl_state_future.done():
            return
        if not self._amcl_state_client.service_is_ready():
            with self._lock:
                self._amcl_lifecycle = {"id": 0, "label": "unavailable"}
            return
        self._amcl_state_future = self._amcl_state_client.call_async(GetState.Request())
        self._amcl_state_future.add_done_callback(self._on_amcl_state)

    def _on_amcl_state(self, future):
        try:
            current = future.result().current_state
            lifecycle = {"id": int(current.id), "label": current.label}
        except Exception as exc:
            lifecycle = {"id": 0, "label": f"error: {exc}"}
        with self._lock:
            self._amcl_lifecycle = lifecycle

    def snapshot(self):
        with self._lock:
            localization_age = self._age(self._last_localization)
            localization_ready = self._localization_ready and localization_age is not None and localization_age < 2
            if LOCALIZATION_MODE == "amcl":
                localization_ready = localization_ready and self._amcl_lifecycle["id"] == State.PRIMARY_STATE_ACTIVE
            error_m = yaw_error = None
            if self._pose is not None and self._ground_truth_pose is not None:
                error_m = math.hypot(self._pose[0] - self._ground_truth_pose[0], self._pose[1] - self._ground_truth_pose[1])
                yaw_error = abs(math.atan2(math.sin(self._pose[2] - self._ground_truth_pose[2]), math.cos(self._pose[2] - self._ground_truth_pose[2])))
            odom_error_m = odom_yaw_error = None
            if self._odom_pose is not None and self._ground_truth_pose is not None:
                odom_error_m = math.hypot(self._odom_pose[0] - self._ground_truth_pose[0], self._odom_pose[1] - self._ground_truth_pose[1])
                odom_yaw_error = abs(math.atan2(math.sin(self._odom_pose[2] - self._ground_truth_pose[2]), math.cos(self._odom_pose[2] - self._ground_truth_pose[2])))
            relocalization_elapsed = None if self._relocalization_started is None else round(time.monotonic() - self._relocalization_started, 3)
            result = {
                "simulation": True,
                "simulation_backend": "gazebo_fortress_planar_base",
                "localization_mode": "amcl_laser_scan" if LOCALIZATION_MODE == "amcl" else "gazebo_ground_truth_odom",
                "map_active": self._map_active,
                "map_ready": self._map is not None,
                "localization_ready": localization_ready,
                "localization_age_s": localization_age,
                "localization_covariance": None if self._localization_covariance is None else dict(self._localization_covariance),
                "localization_error_m": None if error_m is None else round(error_m, 4),
                "localization_yaw_error_rad": None if yaw_error is None else round(yaw_error, 4),
                "odometry_mode": ODOMETRY_MODE,
                "odometry_linear_scale": ODOMETRY_LINEAR_SCALE if ODOMETRY_MODE == "deterministic_scale" else 1.0,
                "odometry_yaw_scale": ODOMETRY_YAW_SCALE if ODOMETRY_MODE == "deterministic_scale" else 1.0,
                "odometry_drift_error_m": None if odom_error_m is None else round(odom_error_m, 4),
                "odometry_drift_yaw_rad": None if odom_yaw_error is None else round(odom_yaw_error, 4),
                "localization_state": "relocalizing" if self._relocalizing else ("tracking" if localization_ready else "unavailable"),
                "relocalization": {
                    "attempts": self._relocalization_attempts,
                    "elapsed_s": relocalization_elapsed,
                    "stable_samples": self._relocalization_stable_samples,
                    "last_error": self._relocalization_last_error,
                    "scan_active": self._relocalization_scan_active,
                    "scan_seconds": RELOCALIZATION_SCAN_SECONDS,
                    "scan_speed": RELOCALIZATION_SCAN_SPEED,
                },
                "ground_truth_age_s": self._age(self._last_ground_truth),
                "ground_truth_pose": None if self._ground_truth_pose is None else {
                    "x": self._ground_truth_pose[0],
                    "y": self._ground_truth_pose[1],
                    "yaw": self._ground_truth_pose[2],
                },
                "amcl_lifecycle": dict(self._amcl_lifecycle),
                "nav2_lifecycle": dict(self._nav2_lifecycle),
                "scan_age_s": self._age(self._last_scan),
                "odom_age_s": self._age(self._last_odom),
                "map_age_s": self._age(self._last_map),
                "pose": None if self._pose is None else {"x": self._pose[0], "y": self._pose[1], "yaw": self._pose[2]},
                "navigation": dict(self._goal),
            }
        result["nav2_action_server_ready"] = self._action.server_is_ready()
        result["nav2_ready"] = result["nav2_action_server_ready"] and result["nav2_lifecycle"]["id"] == State.PRIMARY_STATE_ACTIVE
        result["ready"] = result["map_ready"] and result["localization_ready"] and result["nav2_ready"] and result["scan_age_s"] is not None and result["scan_age_s"] < 2 and result["odom_age_s"] is not None and result["odom_age_s"] < 2
        return result

    def relocalize(self):
        if LOCALIZATION_MODE != "amcl":
            raise ValueError("relocalization_not_supported")
        with self._lock:
            if self._goal_handle is not None:
                raise ValueError("navigation_goal_active")
        if not self._relocalize_client.wait_for_service(timeout_sec=3.0):
            raise ValueError("relocalization_service_unavailable")
        with self._lock:
            self._relocalizing = True
            self._relocalization_started = time.monotonic()
            self._relocalization_stable_samples = 0
            self._relocalization_attempts += 1
            self._relocalization_last_error = ""
            self._localization_ready = False
        event, outcome = threading.Event(), {}
        future = self._relocalize_client.call_async(Empty.Request())
        def completed(done):
            try: done.result()
            except Exception as exc: outcome["error"] = str(exc)
            event.set()
        future.add_done_callback(completed)
        if not event.wait(5):
            outcome["error"] = "relocalization_request_timeout"
        if outcome.get("error"):
            with self._lock:
                self._relocalizing = False
                self._relocalization_last_error = outcome["error"]
            raise ValueError(outcome["error"])
        return {**self.snapshot(), "state": "relocalizing"}

    def _drive_relocalization_scan(self):
        with self._lock:
            elapsed = None if self._relocalization_started is None else time.monotonic() - self._relocalization_started
            scanning = self._relocalizing and elapsed < RELOCALIZATION_SCAN_SECONDS
            should_stop = self._relocalization_scan_active and not scanning
            self._relocalization_scan_active = scanning
        if scanning:
            message = Twist()
            message.angular.z = RELOCALIZATION_SCAN_SPEED
            self._cmd_vel_pub.publish(message)
        elif should_stop:
            self._cmd_vel_pub.publish(Twist())

    def set_map_active(self, active):
        with self._lock: self._map_active = bool(active)

    def _goal_is_free(self, x, y):
        with self._lock: grid = self._map
        if grid is None: return False, "map_not_ready"
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height: return False, "goal_outside_map"
        value = grid.data[row * grid.info.width + col]
        return (value >= 0 and value < 65), "goal_occupied_or_unknown"

    def navigate(self, x, y, yaw):
        status = self.snapshot()
        if not status["ready"]: raise ValueError(f"navigation_not_ready: {status}")
        free, reason = self._goal_is_free(x, y)
        if not free: raise ValueError(reason)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        event, outcome = threading.Event(), {}
        future = self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        def accepted(done):
            try: outcome["handle"] = done.result()
            except Exception as exc: outcome["error"] = str(exc)
            event.set()
        future.add_done_callback(accepted)
        if not event.wait(5): raise ValueError("nav2_goal_response_timeout")
        handle = outcome.get("handle")
        if handle is None or not handle.accepted: raise ValueError(outcome.get("error", "nav2_goal_rejected"))
        goal_id = str(uuid.uuid4())
        with self._lock:
            self._goal_handle = handle
            self._goal = {"state": "navigating", "goal_id": goal_id, "target": {"x": x, "y": y, "yaw": yaw}, "error": ""}
        result = handle.get_result_async()
        result.add_done_callback(lambda done: self._on_result(goal_id, done))
        return dict(self._goal)

    def _on_feedback(self, feedback):
        with self._lock:
            if self._goal.get("state") == "navigating":
                self._goal["distance_remaining"] = float(feedback.feedback.distance_remaining)

    def _on_result(self, goal_id, future):
        try:
            status = int(future.result().status)
        except Exception as exc:
            update = {"state": "failed", "error": str(exc)}
        else:
            with self._lock:
                cancel_requested = self._cancel_requested_goal_id == goal_id
            update = terminal_goal_update(status, cancel_requested=cancel_requested)
        with self._lock:
            if self._goal.get("goal_id") == goal_id:
                self._goal.update(update)
                self._goal_handle = None
                self._cancel_requested_goal_id = None

    def cancel(self):
        with self._lock:
            handle = self._goal_handle
            goal_id = self._goal.get("goal_id")
        if handle is None: return {"state": "idle", "canceled": False}
        with self._lock:
            self._cancel_requested_goal_id = goal_id
        event = threading.Event()
        try:
            future = handle.cancel_goal_async()
        except Exception:
            with self._lock:
                if self._cancel_requested_goal_id == goal_id:
                    self._cancel_requested_goal_id = None
            raise
        future.add_done_callback(lambda _: event.set())
        if not event.wait(3):
            with self._lock:
                if self._cancel_requested_goal_id == goal_id:
                    self._cancel_requested_goal_id = None
            raise ValueError("nav2_cancel_timeout")
        with self._lock: self._goal["state"] = "canceling"
        return {"state": "canceling", "canceled": True, "goal_id": goal_id}

    def _publish_mapping(self):
        with self._lock: grid, pose, active = self._map, self._pose, self._map_active
        if not active or grid is None or pose is None or not self.snapshot()["localization_ready"]: return
        points = []
        stride = max(1, int(max(grid.info.width, grid.info.height) / 160))
        for row in range(0, grid.info.height, stride):
            for col in range(0, grid.info.width, stride):
                if grid.data[row * grid.info.width + col] >= 65:
                    points.append((grid.info.origin.position.x + (col + 0.5) * grid.info.resolution, grid.info.origin.position.y + (row + 0.5) * grid.info.resolution, 0.0))
        payload = bytearray(struct.pack("<fffBI", pose[0], pose[1], pose[2], 0x03, len(points)))
        for point in points: payload.extend(struct.pack("<fff", *point))
        message = UInt8MultiArray(); message.data = list(payload); self._mapping_pub.publish(message)

    def _tick(self):
        self._publish_mapping()
        message = String(); message.data = json.dumps(self.snapshot(), separators=(",", ":")); self._status_pub.publish(message)


class Bundle:
    def __init__(self, node): self.node, self.navigation_started = node, False
    def tools(self):
        return [
            {"name": "navigation_map", "type": "sensor", "multiInstance": False, "description": "SIMULATION ONLY — Gazebo room map and robot pose", "inputSchema": {"type": "object", "properties": {}}, "topic_out": [{"topic": f"{ROOT}/mapping", "format": "sensor/mapping"}]},
            {"name": "navigation", "type": "actuator", "multiInstance": False, "description": "SIMULATION ONLY — Nav2 goal control for the Gazebo planar base; not G1 biped locomotion", "topic_out": [{"topic": f"{ROOT}/navigation/status", "format": "data/json"}], "inputSchema": {"type": "object", "properties": {"action": {"type": "string", "enum": ["info", "start", "stop", "navigate_to_pose", "cancel", "relocalize"]}, "x": {"type": "number", "minimum": -4.5, "maximum": 4.5}, "y": {"type": "number", "minimum": -3.5, "maximum": 3.5}, "yaw": {"type": "number", "minimum": -3.141592653589793, "maximum": 3.141592653589793}}, "required": ["action"], "x-action-params": {"navigate_to_pose": {"params": ["x", "y", "yaw"], "description": "Plan and drive to a free map pose"}, "cancel": {"params": [], "description": "Cancel the active goal"}, "relocalize": {"params": [], "description": "Reinitialize AMCL globally after the robot pose becomes unknown"}}}},
        ]
    def dispatch(self, name, args):
        action = args.get("action", name)
        if name == "navigation_map":
            if action == "start": self.node.set_map_active(True)
            elif action == "stop": self.node.set_map_active(False)
            elif action != "info": return None
            return {**self.node.snapshot(), "state": "running" if self.node.snapshot()["map_active"] else "idle", "topic_out": [{"topic": f"{ROOT}/mapping", "format": "sensor/mapping"}]}
        if name != "navigation": return None
        if action == "start": self.navigation_started = True
        elif action == "stop": self.navigation_started = False; self.node.cancel()
        elif action == "navigate_to_pose":
            if not self.navigation_started: raise ValueError("navigation must be started before goals")
            return self.node.navigate(float(args["x"]), float(args["y"]), float(args.get("yaw", 0)))
        elif action == "cancel": return self.node.cancel()
        elif action == "relocalize": return self.node.relocalize()
        elif action != "info": return None
        snapshot = self.node.snapshot()
        state = "relocalizing" if snapshot["localization_state"] == "relocalizing" else ("ready" if self.navigation_started else "idle")
        return {**snapshot, "state": state}
    def status(self): return {"ok": True, "name": "gazebo-navigation-bundle", **self.node.snapshot()}


bundle = None
def response(handler, status, payload):
    body = json.dumps(payload, ensure_ascii=False).encode(); handler.send_response(status); handler.send_header("Content-Type", "application/json"); handler.send_header("Content-Length", str(len(body))); handler.end_headers(); handler.wfile.write(body)
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass
    def do_GET(self): response(self, 200, bundle.status()) if self.path == "/health" else response(self, 404, {"ok": False})
    def do_POST(self):
        if self.path != "/mcp": return response(self, 404, {"ok": False})
        rpc = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0")))); rid = rpc.get("id")
        try:
            method, params = rpc.get("method"), rpc.get("params") or {}
            if method == "initialize": result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}}, "serverInfo": {"name": "gazebo-navigation-bundle", "version": "0.1.0"}}
            elif method == "tools/list": result = {"tools": bundle.tools()}
            elif method == "tools/call":
                value = bundle.dispatch(params.get("name", ""), params.get("arguments") or {})
                if value is None: raise KeyError("unknown tool or action")
                result = {"content": [{"type": "text", "text": json.dumps(value, ensure_ascii=False)}]}
            else: raise KeyError("method not found")
            response(self, 200, {"jsonrpc": "2.0", "id": rid, "result": result})
        except (ValueError, KeyError) as exc: response(self, 200, {"jsonrpc": "2.0", "id": rid, "error": {"code": -32602, "message": str(exc)}})


def register():
    payload = json.dumps({"name": "Simulated Navigation (Gazebo)", "transport": "http", "url": MCP_ADVERTISE_URL, "category": "driver"}).encode()
    context = ssl.create_default_context(); context.check_hostname = False; context.verify_mode = ssl.CERT_NONE
    while rclpy.ok():
        try:
            urllib_request.urlopen(urllib_request.Request(f"{AGENT_CORE_URL}/api/mcp", data=payload, headers={"Content-Type": "application/json"}, method="POST"), timeout=5, context=context).read(); time.sleep(30)
        except Exception as exc: print(f"[register] {exc}", flush=True); time.sleep(5)


def main():
    global bundle
    rclpy.init(); node = NavigationNode(); bundle = Bundle(node)
    server = ThreadingHTTPServer(("0.0.0.0", MCP_PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start(); threading.Thread(target=register, daemon=True).start()
    try: rclpy.spin(node)
    finally: server.shutdown(); node.destroy_node(); rclpy.shutdown()
