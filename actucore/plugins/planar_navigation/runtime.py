"""Real ROS 2 adapter. Callbacks never block on service/action futures."""

import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String, UInt8MultiArray
from tf2_ros import Buffer, TransformListener

from .model import Rejected, ProposalGate, fresh, number
from .parameters import parameters
from .semantic import decode_rgb


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1) > 0.05:
        raise Rejected("invalid_pose", "invalid quaternion")
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Runtime(Node):
    def __init__(self, cfg, executor, complete):
        super().__init__("planar_card", namespace=cfg["namespace"])
        self.cfg, self.executor, self.complete = cfg, executor, complete
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.process = None
        self.work = tempfile.TemporaryDirectory(prefix="planar-runtime-")
        self.mode = "idle"
        self.scan = self.odom = self.grid = self.costmap = self.rgb = None
        self.scan_at = self.odom_at = self.driver_at = 0.0
        self.driver = {}
        self.pose = None
        self.pose_ok = False
        self.localized_after = 0.0
        self.pose_covariance_ok = False
        self.latch = ""
        self.diagnostic = "waiting_for_sensors"
        self.path = []
        self.last_view = 0.0
        self.states = {}
        self.state_clients = {name: self.create_client(GetState, name + "/get_state")
                              for name in ("amcl", "map_server", "planner_server",
                                           "controller_server", "bt_navigator")}
        self.state_pending = {}
        self.nav_handle = self.nav_future = None
        self.pending_terminal = None
        self.goal_pose = None
        self.nav_token = 0
        self.pending_stop_at = 0.0
        self.stopped_event = threading.Event()
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        sensor = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.proposal_pub = self.create_publisher(String, "nav2/velocity_proposal", 1)
        self.preview_proposal_pub = self.create_publisher(String, "proposal_preview", 1)
        self.status_pub = self.create_publisher(String, "status", 1)
        self.image_pub = self.create_publisher(CompressedImage, "map_view", 1)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.gate = ProposalGate(self._publish, cfg)
        self.create_subscription(LaserScan, cfg["scan_topic"], self._scan, sensor)
        self.create_subscription(Odometry, cfg["odom_topic"], self._odom, sensor)
        self.create_subscription(OccupancyGrid, "map", self._map, latched)
        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self._costmap, sensor)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._localized, 1)
        self.create_subscription(RosPath, "plan", self._path, sensor)
        self.create_subscription(Twist, "controller_cmd_vel", self._command, 1)
        if cfg["rgb_topic"]:
            self.create_subscription(UInt8MultiArray, cfg["rgb_topic"], self._rgb, sensor)
        if cfg["driver_status_topic"]:
            self.create_subscription(String, cfg["driver_status_topic"], self._driver, 1)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(0.05, self._watch)
        self.create_timer(1 / cfg["proposal_hz"], self.gate.tick)
        executor.add_node(self)

    def _scan(self, msg):
        with self.lock:
            stamp = seconds(msg.header.stamp)
            if self.scan and stamp <= seconds(self.scan.header.stamp):
                return
            if (not msg.header.frame_id or not 2 <= len(msg.ranges) <= 100000
                    or not math.isfinite(msg.angle_increment) or msg.angle_increment <= 0
                    or not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max)
                    or not 0 <= msg.range_min < msg.range_max
                    or not any(math.isfinite(v) and msg.range_min <= v <= msg.range_max
                               for v in msg.ranges)):
                self.diagnostic = "invalid_scan"
                self.scan = None
                return
            self.scan, self.scan_at = msg, time.monotonic()

    def _odom(self, msg):
        with self.lock:
            stamp = seconds(msg.header.stamp)
            if self.odom and stamp <= seconds(self.odom.header.stamp):
                return
            p = msg.pose.pose.position
            v = msg.twist.twist
            try:
                angle = yaw(msg.pose.pose.orientation)
                if (msg.header.frame_id != self.cfg["odom_frame"]
                        or msg.child_frame_id != self.cfg["base_frame"]
                        or not all(math.isfinite(x) for x in (p.x, p.y, v.linear.x, v.angular.z))
                        or not all(math.isfinite(x) for x in msg.pose.covariance)
                        or any(msg.pose.covariance[i] < 0 for i in (0, 7, 35))):
                    raise ValueError("invalid odometry")
                if self.odom:
                    prev = self.odom.pose.pose.position
                    dt = stamp - seconds(self.odom.header.stamp)
                    dyaw = math.atan2(math.sin(angle-yaw(self.odom.pose.pose.orientation)),
                                     math.cos(angle-yaw(self.odom.pose.pose.orientation)))
                    if (math.hypot(p.x-prev.x, p.y-prev.y) > 0.3 + dt
                            or abs(dyaw) > 0.4 + 2*dt):
                        self.latch = "odom_discontinuity"
                self.odom, self.odom_at = msg, time.monotonic()
            except (ValueError, Rejected):
                self.odom = None
                self.diagnostic = "invalid_odom"

    def _map(self, msg):
        with self.lock:
            if (msg.header.frame_id == self.cfg["map_frame"]
                    and 0 < msg.info.width * msg.info.height <= 16_000_000
                    and len(msg.data) == msg.info.width * msg.info.height
                    and math.isfinite(msg.info.resolution) and msg.info.resolution > 0):
                self.grid = msg

    def _costmap(self, msg):
        with self.lock:
            if (msg.header.frame_id == self.cfg["map_frame"]
                    and 0 < msg.info.width * msg.info.height <= 16_000_000
                    and len(msg.data) == msg.info.width * msg.info.height
                    and math.isfinite(msg.info.resolution) and msg.info.resolution > 0):
                self.costmap = (msg, time.monotonic())

    def _path(self, msg):
        with self.lock:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses[:10000]]

    def _localized(self, msg):
        with self.lock:
            cov = msg.pose.covariance
            self.pose_covariance_ok = (
                seconds(msg.header.stamp) >= self.localized_after > 0
                and msg.header.frame_id == self.cfg["map_frame"]
                and all(math.isfinite(v) and 0 <= v <= limit
                        for v, limit in ((cov[0], 0.25), (cov[7], 0.25), (cov[35], 0.09))))

    def _driver(self, msg):
        try:
            data = json.loads(msg.data)
            if (data.get("schema") != "phanthy.navigation.execution_status.v1"
                    or not isinstance(data.get("source_stamp_ns"), int)):
                return
            with self.lock:
                if data["source_stamp_ns"] <= self.driver.get("source_stamp_ns", 0):
                    return
                self.driver, self.driver_at = data, time.monotonic()
        except (ValueError, TypeError, AttributeError):
            return

    def _rgb(self, msg):
        try:
            frame = decode_rgb(bytes(msg.data))
            with self.lock:
                if not self.rgb or frame[0] > self.rgb[0]:
                    self.rgb = (*frame, time.monotonic())
        except Rejected as exc:
            with self.lock:
                self.diagnostic = exc.code

    def _command(self, msg):
        with self.lock:
            if self.nav_handle is None or self.pending_terminal:
                return
        try:
            self.gate.command(msg.linear.x, msg.angular.z)
        except Rejected:
            self.gate.health("invalid_controller_velocity")

    def _publish(self, payload):
        msg = String(data=json.dumps(payload, allow_nan=False))
        # Shadow output is deliberately a different topic: the existing Driver
        # executes shadow_only proposals, so that flag is NOT an execution switch.
        target = self.proposal_pub if self.cfg["execution_enabled"] else self.preview_proposal_pub
        target.publish(msg)

    def blocker(self):
        with self.lock:
            now, wall = time.monotonic(), time.time()
            if self.latch:
                return self.latch
            for name, msg, at in (("scan", self.scan, self.scan_at), ("odom", self.odom, self.odom_at)):
                if msg is None or not fresh(seconds(msg.header.stamp), at, wall, now, self.cfg["sensor_max_age"]):
                    return name + "_stale"
            try:
                self.tf.lookup_transform(self.cfg["base_frame"], self.scan.header.frame_id,
                                         Time.from_msg(self.scan.header.stamp))
                self.tf.lookup_transform(self.cfg["odom_frame"], self.cfg["base_frame"],
                                         Time.from_msg(self.odom.header.stamp))
                if self.mode in ("mapping", "localization"):
                    transform = self.tf.lookup_transform(
                        self.cfg["map_frame"], self.cfg["base_frame"],
                        Time.from_msg(self.odom.header.stamp))
                    p, q = transform.transform.translation, transform.transform.rotation
                    self.pose = {"x": p.x, "y": p.y, "yaw": yaw(q)}
                    if self.mode == "localization" and not self.pose_covariance_ok:
                        return "localization_unconfirmed"
            except Exception:
                return "tf_unavailable"
            if self.mode in ("mapping", "localization"):
                if not self.process or self.process.poll() is not None:
                    return "algorithm_exited"
            if self.mode == "localization":
                if any(state != 3 or now-at > 3 for state, at in self.states.values()) or len(self.states) != 5:
                    return "lifecycle_not_active"
                if not self.nav.server_is_ready():
                    return "navigator_unavailable"
                if not self.costmap or not fresh(seconds(self.costmap[0].header.stamp),
                                                self.costmap[1], wall, now, 2):
                    return "costmap_stale"
            if self.cfg["execution_enabled"]:
                data = self.driver
                if (not fresh(data.get("source_stamp_ns", 0) / 1e9, self.driver_at, wall, now, 0.5)
                        or data.get("safety_ready") is not True):
                    return "driver_not_ready"
                if data.get("nav_id") not in (None, "", self.gate.nav_id):
                    return "driver_control_conflict"
            return ""

    def _watch(self):
        blocker = self.blocker()
        self.gate.health(blocker)
        completed = None
        with self.lock:
            self.pose_ok = not blocker and self.mode == "localization"
            self.diagnostic = blocker
            terminal = self.pending_terminal
            if terminal and self._stop_confirmed():
                nav_id, status = self.gate.nav_id, terminal
                self.gate.finish(status)
                self.pending_terminal = None
                self.stopped_event.set()
                completed = {"action_id": nav_id, "nav_id": nav_id,
                             "status": status, "terminal_confirmed": True,
                             "physical_execution": self.cfg["execution_enabled"]}
            now = time.monotonic()
            if now - self.last_view >= 1:
                self.last_view = now
                if self.mode == "localization":
                    for name, client in self.state_clients.items():
                        pending = self.state_pending.get(name)
                        if pending and now - pending[1] > 2:
                            client.remove_pending_request(pending[0])
                            self.state_pending.pop(name, None)
                        if name not in self.state_pending and client.service_is_ready():
                            future = client.call_async(GetState.Request())
                            self.state_pending[name] = (future, now)
                            future.add_done_callback(
                                lambda f, n=name: self._lifecycle(f, n))
                self.status_pub.publish(String(data=json.dumps(self.status())))
                try:
                    self._render()
                except (ValueError, OSError):
                    self.diagnostic = "map_preview_failed"
        if completed:
            self.complete(completed)

    def _lifecycle(self, future, name):
        with self.lock:
            if self.state_pending.get(name, (None,))[0] is not future:
                return
            self.state_pending.pop(name, None)
            try:
                self.states[name] = (future.result().current_state.id, time.monotonic())
            except Exception:
                self.states.pop(name, None)

    def _stop_confirmed(self):
        if self.nav_handle is not None or self.nav_future is not None:
            return False
        if self.cfg["execution_enabled"]:
            data = self.driver
            return (data.get("nav_id") == self.gate.nav_id
                    and data.get("stop_confirmed") is True
                    and data.get("source_stamp_ns", 0)/1e9 > self.pending_stop_at
                    and fresh(data.get("source_stamp_ns", 0)/1e9, self.driver_at,
                              time.time(), time.monotonic(), 0.5))
        return True  # Planner-only completion, never advertised as physical stop.

    def status(self):
        with self.lock:
            return {"state": self.mode, "ready": self.pose_ok,
                    "reason": self.diagnostic, "nav_status": self.gate.status,
                    "nav_id": self.gate.nav_id, "pose": self.pose,
                    "execution_enabled": self.cfg["execution_enabled"],
                    "terminal_pending": self.pending_terminal}

    def launch(self, mode, map_yaml=""):
        self.end_stack()
        if self.cancelled.is_set():
            raise Rejected("cancelled", "card stopped")
        if mode == "localization" and not self.cfg["footprint"]:
            raise Rejected("footprint_required", "configure measured robot footprint")
        values = parameters(self.cfg, map_yaml)
        values["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = str(
            Path(__file__).with_name("navigate.xml"))
        # Fully-qualified keys work with an arbitrary robot namespace.
        values = {"/" + self.cfg["namespace"] + "/" + key: val for key, val in values.items()}
        params = Path(self.work.name) / "params.yaml"
        params.write_text(yaml.safe_dump(values))
        with self.lock:
            if self.cancelled.is_set():
                raise Rejected("cancelled", "card stopped")
            self.grid = self.costmap = None
            self.states.clear()
            for name, (future, _) in self.state_pending.items():
                self.state_clients[name].remove_pending_request(future)
            self.state_pending.clear()
            self.pose_covariance_ok = self.pose_ok = False
            self.localized_after = 0
            self.mode = mode
            self.process = subprocess.Popen(
                ["ros2", "launch", str(Path(__file__).with_name("stack.launch.py")),
                 "mode:=" + mode, "namespace:=" + self.cfg["namespace"],
                 "params:=" + str(params)], start_new_session=True)

    def end_stack(self):
        with self.lock:
            process, self.process = self.process, None
            self.mode = "idle"
            self.pose_ok = False
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)

    def save_map(self, prefix):
        # map_saver_server is intentionally short lived and has its own lifecycle
        # in CLI; output is verified before the mapping group is stopped.
        with self.lock:
            if self.mode != "mapping" or self.grid is None or not any(v > 0 for v in self.grid.data):
                raise Rejected("map_not_ready", "no occupied map cells received")
        proc = subprocess.Popen(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix),
             "--ros-args", "-p", "map_subscribe_transient_local:=true",
             "-p", "save_map_timeout:=10.0",
             "-r", "map:=/" + self.cfg["namespace"] + "/map"],
            start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while proc.poll() is None:
                if self.cancelled.wait(0.05) or time.monotonic() > deadline:
                    raise Rejected("map_save_failed", "map save cancelled or timed out")
            if proc.returncode:
                raise Rejected("map_save_failed", "map_saver failed; mapping retained")
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)

    def relocalize(self, pose):
        with self.lock:
            if self.mode != "localization":
                raise Rejected("map_not_loaded", "load a map first")
            if not self.initial_pub.get_subscription_count() or self.states.get("amcl", (0,))[0] != 3:
                raise Rejected("runtime_not_ready", "wait for AMCL activation before relocalizing")
            self.pose_covariance_ok = self.pose_ok = False
            self.localized_after = time.time()
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.cfg["map_frame"]
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose["x"], pose["y"]
            msg.pose.pose.orientation.z = math.sin(pose["yaw"]/2)
            msg.pose.pose.orientation.w = math.cos(pose["yaw"]/2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.09
            self.initial_pub.publish(msg)
        return {"status": "localizing", "ready": False}

    def check_goal(self, pose):
        problem = self.blocker()
        if problem or self.mode != "localization":
            raise Rejected("navigation_not_ready", problem or "map_not_loaded")
        with self.lock:
            msg, _ = self.costmap
            info = msg.info
            angle = yaw(info.origin.orientation)
            dx, dy = pose["x"]-info.origin.position.x, pose["y"]-info.origin.position.y
            x = math.floor((math.cos(angle)*dx+math.sin(angle)*dy)/info.resolution)
            y = math.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/info.resolution)
            if not 0 <= x < info.width or not 0 <= y < info.height:
                raise Rejected("goal_outside_costmap", "goal is outside the saved map")
            cost = msg.data[y*info.width+x]
            if cost < 0 or cost >= 99:
                raise Rejected("goal_in_collision", "goal is occupied, inscribed or unknown")

    def navigate(self, nav_id, pose):
        self.check_goal(pose)
        with self.lock:
            self.gate.begin(nav_id)
            self.nav_token += 1
            token = self.nav_token
            self.goal_pose = pose
            self.stopped_event.clear()
            msg = NavigateToPose.Goal()
            msg.pose.header.frame_id = self.cfg["map_frame"]
            msg.pose.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose["x"], pose["y"]
            msg.pose.pose.orientation.z = math.sin(pose["yaw"]/2)
            msg.pose.pose.orientation.w = math.cos(pose["yaw"]/2)
            self.nav_future = self.nav.send_goal_async(msg)
            self.nav_future.add_done_callback(lambda f: self._accepted(f, token))

    def _accepted(self, future, token):
        with self.lock:
            self.nav_future = None
            try:
                handle = future.result()
                if not handle.accepted:
                    self._terminal("aborted")
                    return
                self.nav_handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, token))
                if self.pending_terminal or self.gate.status == "paused" or token != self.nav_token:
                    handle.cancel_goal_async()
            except Exception:
                self._terminal("aborted")

    def _result(self, future, token):
        with self.lock:
            if token != self.nav_token:
                return
            self.nav_handle = None
            if self.gate.status == "paused" and not self.pending_terminal:
                return
            try:
                state = future.result().status
                terminal = "arrived" if state == 4 else "cancelled" if state == 5 else "aborted"
            except Exception:
                terminal = "aborted"
            self._terminal(self.pending_terminal or terminal)

    def _terminal(self, state):
        if self.pending_terminal:
            return  # A retry must not move the required stop-receipt time forward.
        self.pending_terminal = state
        self.pending_stop_at = time.time()
        # Driver receives the terminal intent now, but ACP waits for its receipt.
        self.gate.hold(state, "waiting_for_stop_confirmation")

    def stop_navigation(self):
        with self.lock:
            if not self.gate.nav_id:
                return True
            self._terminal("cancelled")
            if self.nav_handle:
                self.nav_handle.cancel_goal_async()
        return self.stopped_event.wait(5)

    def pause(self):
        with self.lock:
            if not self.gate.nav_id or self.pending_terminal:
                raise Rejected("navigation_inactive", "no pausable task")
            if self.gate.status != "paused":
                self.pending_stop_at = time.time()
            self.gate.hold("paused", "manual_pause")
            if self.nav_handle:
                self.nav_handle.cancel_goal_async()

    def resume(self):
        self.check_goal(self.goal_pose or {})
        with self.lock:
            if self.gate.status != "paused" or not self._stop_confirmed():
                raise Rejected("pause_not_confirmed", "wait for navigator and Driver stop confirmation")
            nav_id, pose = self.gate.nav_id, self.goal_pose
            self.gate.nav_id = None
            self.navigate(nav_id, pose)

    def capture_frame(self):
        after = time.time()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not self.cancelled.wait(0.05):
            with self.lock:
                if not self.pose_ok or not self.rgb:
                    continue
                stamp, frame, jpeg, at = self.rgb
                if stamp < after or not fresh(stamp, at, time.time(), time.monotonic(), 0.5):
                    continue
                try:
                    when = Time(nanoseconds=int(stamp*1e9))
                    self.tf.lookup_transform(self.cfg["base_frame"], frame, when)
                    transform = self.tf.lookup_transform(
                        self.cfg["map_frame"], self.cfg["base_frame"], when)
                    p, q = transform.transform.translation, transform.transform.rotation
                    return {"x": p.x, "y": p.y, "yaw": yaw(q)}, stamp, jpeg
                except Exception:
                    continue
        raise Rejected("sensor_timeout", "no fresh RGB and map pose at the same source time")

    def _render(self):
        from PIL import Image, ImageDraw
        image = Image.new("RGB", (640, 640), "#191919")
        draw = ImageDraw.Draw(image)
        grid = self.costmap[0] if self.mode == "localization" and self.costmap else self.grid
        if grid is not None:
            w, h, res = grid.info.width, grid.info.height, grid.info.resolution
            scale = min(600/w, 550/h)
            # Bound work by output pixels, not by the number of full-map cells.
            small = Image.frombytes("L", (w, h), bytes(grid.data)).resize(
                (max(1, int(w*scale)), max(1, int(h*scale))), Image.NEAREST)
            palette = []
            for value in range(256):
                palette.extend((70, 70, 70) if value > 100 else
                               (220, 65, 50) if value >= (99 if self.mode == "localization" else 65)
                               else (225, 225, 225))
            small.putpalette(palette)
            small = small.convert("RGB").transpose(Image.FLIP_TOP_BOTTOM)
            image.paste(small, (20, 50))
            origin = grid.info.origin
            angle = yaw(origin.orientation)
            def xy(x, y):
                dx, dy = x-origin.position.x, y-origin.position.y
                return (20+(math.cos(angle)*dx+math.sin(angle)*dy)/res*scale,
                        50+small.height-(-math.sin(angle)*dx+math.cos(angle)*dy)/res*scale)
            if len(self.path) > 1:
                draw.line([xy(x, y) for x, y in self.path], fill="#008800", width=2)
            if self.scan and not self.diagnostic:
                try:
                    transform = self.tf.lookup_transform(
                        self.cfg["map_frame"], self.scan.header.frame_id,
                        Time.from_msg(self.scan.header.stamp))
                    pos = transform.transform.translation
                    heading = yaw(transform.transform.rotation)
                    stride = max(1, len(self.scan.ranges)//1200)
                    for i in range(0, len(self.scan.ranges), stride):
                        distance = self.scan.ranges[i]
                        if not math.isfinite(distance) or not self.scan.range_min <= distance <= self.scan.range_max:
                            continue
                        a = heading + self.scan.angle_min + i*self.scan.angle_increment
                        x, y = xy(pos.x+distance*math.cos(a), pos.y+distance*math.sin(a))
                        draw.point((x, y), fill="#00bbdd")
                except Exception:
                    pass  # No transform: omit the overlay, never invent alignment.
            if self.pose:
                x, y = xy(self.pose["x"], self.pose["y"])
                a = self.pose["yaw"] - angle
                draw.ellipse((x-4, y-4, x+4, y+4), fill="#0066ff")
                draw.line((x, y, x+14*math.cos(a), y-14*math.sin(a)), fill="#0066ff", width=3)
        draw.text((15, 15), "Planar navigation / " + self.mode, fill="white")
        draw.text((15, 615), (self.diagnostic or self.gate.status)[:80], fill="#ffbb55")
        output = io.BytesIO()
        image.save(output, "JPEG", quality=75)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format, msg.data = "jpeg", output.getvalue()
        self.image_pub.publish(msg)

    def close(self):
        self.cancelled.set()
        self.end_stack()
        self.executor.remove_node(self)
        self.destroy_node()
        self.work.cleanup()
