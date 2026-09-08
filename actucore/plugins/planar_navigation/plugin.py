"""Single public product. Stop bypasses the serialized business-operation lock."""

from collections import OrderedDict
import json
import threading
import time
import uuid

from .model import DEFAULTS, RANGES, Rejected, Store, config, goal
from .semantic import ask, choose


ACTIONS = {
    "start": [], "stop": [], "info": [], "config": [],
    "start_mapping": ["map_name"], "stop_mapping": [],
    "list_maps": [], "load_map": ["map_id"], "relocalize": ["x", "y", "yaw"],
    "navigate_to_pose": ["x", "y", "yaw"],
    "pause_navigation": [], "resume_navigation": [], "stop_navigation": [],
    "capture": [], "list_points": [], "delete_point": ["point_id"],
    "navigate": ["query"],
}


def config_schema():
    props = {}
    for key, default in DEFAULTS.items():
        spec = {"default": default, "scope": "shared"}
        if key in RANGES:
            spec.update(type="number", minimum=RANGES[key][0], maximum=RANGES[key][1])
        elif isinstance(default, bool):
            spec["type"] = "boolean"
        elif isinstance(default, list):
            # Canvas supports text entry; config also accepts native JSON arrays.
            spec.update(type="string", default="[]",
                        description="Measured convex footprint [[x,y],...] in metres; required before loading a map")
        else:
            spec["type"] = "string"
        if key in ("vlm_api_key", "vlm_url"):
            spec.update({"x-sensitive": True})
        if key == "vlm_api_key":
            spec["format"] = "password"
        props[key] = spec
    return {"type": "object", "properties": props, "additionalProperties": False}


class PlanarNavigationPlugin:
    PREFIX = "PlanarSemanticNavigation"

    def __init__(self, plugin_cfg, executor, completion=None):
        self.cfg = config(plugin_cfg)
        self.executor = executor
        self.completion = completion or (lambda _: None)
        self.lock = threading.RLock()
        self.operations = threading.Lock()
        self.runtime = None
        self.store = None
        self.map_item = None
        self.mapping_name = ""
        self.epoch = 0
        self.starting = False
        self.last_error = ""
        self.receipts = OrderedDict()

    def topics(self):
        cfg = self.cfg
        root = "/" + cfg["namespace"]
        inputs = [
            {"port": "scan", "topic": cfg["scan_topic"], "format": "sensor/lidar-2d",
             "ros_type": "sensor_msgs/msg/LaserScan", "required": True},
            {"port": "odom", "topic": cfg["odom_topic"], "format": "data/json",
             "ros_type": "nav_msgs/msg/Odometry", "required": True},
            {"port": "rgb", "topic": cfg["rgb_topic"],
             "format": "application/vnd.phanthy.sensor-envelope.v1",
             "schema": "phanthy.sensor.camera_rgb_frame.v1", "required": False},
        ]
        if cfg["driver_status_topic"]:
            inputs.append({"port": "execution_status", "topic": cfg["driver_status_topic"],
                           "format": "data/json", "required": cfg["execution_enabled"]})
        outputs = [
            {"port": "map_view", "topic": root+"/map_view", "format": "image/jpeg"},
            {"port": "status", "topic": root+"/status", "format": "data/json"},
            {"port": "velocity_proposal",
             "topic": root + ("/nav2/velocity_proposal" if cfg["execution_enabled"]
                             else "/proposal_preview"), "format": "data/json",
             "schema": "phanthy.navigation.velocity_proposal.v1",
             "description": "Preview only; DO NOT wire to Driver" if not cfg["execution_enabled"]
                            else "Connect to sole Driver navigation executor"},
        ]
        # No preview port may be mistaken for an actionable proposal stream.
        if not cfg["execution_enabled"]:
            outputs = outputs[:2]
        return {"topic_in": inputs, "topic_out": outputs}

    def get_tools(self):
        props = {
            "action": {"type": "string", "enum": list(ACTIONS)},
            "map_name": {"type": "string", "minLength": 1, "maxLength": 128},
            "map_id": {"type": "string"}, "point_id": {"type": "string"},
            "query": {"type": "string", "minLength": 1, "maxLength": 1000},
            "x": {"type": "number", "minimum": -10000, "maximum": 10000},
            "y": {"type": "number", "minimum": -10000, "maximum": 10000},
            "yaw": {"type": "number", "minimum": -3.141592653589793,
                    "maximum": 3.141592653589793},
            "input_topic": {"type": "string"},
            "input_topics": {"type": "array", "items": {"type": "string"}},
            "input_bindings": {"type": "array", "items": {"type": "object"}},
            "instance_id": {"type": "string"},
        }
        return [{
            "name": self.PREFIX, "type": "processor", "multiInstance": False,
            "description": "二维语义导航：建图、存图、定位、避障与拍照记地点。需要二维扫描和连续里程计。",
            "inputSchema": {
                "type": "object", "properties": props, "required": ["action"],
                "x-action-params": ACTIONS,
                "x-completion": {
                    "actions": ["navigate_to_pose", "navigate"], "timeout": 3600,
                    "passthrough_actions": ["info", "pause_navigation", "resume_navigation",
                                            "stop_navigation", "stop"],
                },
                "x-hooks": {"on_interrupt_navigation": {"action": "stop_navigation"}},
            },
            "configSchema": config_schema(), **self.topics(),
        }]

    def _store(self):
        if self.store is None:
            self.store = Store(self.cfg["data_dir"])
        return self.store

    def _completed(self, result):
        with self.lock:
            self.receipts[result["nav_id"]] = result
            while len(self.receipts) > 128:
                self.receipts.popitem(last=False)
        self.completion({"type": "action_complete", "action_id": result["action_id"],
                         "status": result["status"], "payload": result})

    def info(self):
        with self.lock:
            result = self.runtime.status() if self.runtime else {"state": "idle", "ready": False}
            return {**result, "name": self.PREFIX, "map": self.map_item,
                    "last_error": self.last_error, "starting": self.starting,
                    "config": {k: ("****" if v else "") if k == "vlm_api_key" else v
                               for k, v in self.cfg.items()}, **self.topics()}

    def _wiring(self, args):
        bindings = args.get("input_bindings")
        if bindings:
            if (not isinstance(bindings, list) or
                    any(not isinstance(x, dict) or not isinstance(x.get("port"), str)
                        or not isinstance(x.get("topic"), str) for x in bindings)):
                raise Rejected("invalid_canvas_wiring", "invalid input_bindings")
            ports = [x.get("port") for x in bindings]
            if len(set(ports)) != len(ports):
                raise Rejected("invalid_canvas_wiring", "duplicate ports")
            allowed = {x["port"]: x["topic"] for x in self.topics()["topic_in"]}
            if any(x["port"] not in allowed or x["topic"] != allowed[x["port"]]
                   for x in bindings):
                raise Rejected("invalid_canvas_wiring", "bindings must match configured topics")
            selected = {x.get("topic") for x in bindings}
        else:
            values = args.get("input_topics", [])
            if not isinstance(values, list) or any(not isinstance(v, str) for v in values):
                raise Rejected("invalid_canvas_wiring", "input_topics must be a string array")
            selected = set(values)
            if args.get("input_topic"):
                if not isinstance(args["input_topic"], str):
                    raise Rejected("invalid_canvas_wiring", "input_topic must be a string")
                selected.add(args["input_topic"])
        required = {x["topic"] for x in self.topics()["topic_in"] if x["required"]}
        if self.cfg["rgb_topic"]:
            required.add(self.cfg["rgb_topic"])
        if not required <= selected:
            raise Rejected("invalid_canvas_wiring", "missing configured inputs: " +
                           ", ".join(sorted(required-selected)))

    def _start(self, args):
        self._wiring(args)
        from .runtime import Runtime
        with self.lock:
            if self.runtime:
                return self.info()
            self.starting = True
            epoch = self.epoch
        runtime = None
        try:
            runtime = Runtime(self.cfg, self.executor, self._completed)
            with self.lock:
                if epoch != self.epoch:
                    raise Rejected("cancelled", "start was cancelled by stop")
                self.runtime = runtime
            return self.info()
        except Exception:
            if runtime:
                runtime.close()
            raise
        finally:
            with self.lock:
                self.starting = False

    def stop(self):
        with self.lock:
            self.epoch += 1
            runtime = self.runtime
            if runtime:
                runtime.cancelled.set()
        if runtime and not runtime.stop_navigation():
            self.last_error = "stop_unconfirmed"
            return {"status": "error", "error_code": "stop_unconfirmed",
                    "error": "Nav2 or Driver has not confirmed stopping; retry stop",
                    "terminal_confirmed": False, "retryable": True}
        if runtime:
            runtime.close()
        with self.lock:
            if self.runtime is runtime:
                self.runtime = None
            self.map_item = None
        return {"state": "idle", "status": "stopped", "terminal_confirmed": True,
                "physical_execution": False}

    def dispatch(self, name, args):
        if name != self.PREFIX:
            return None
        try:
            if not isinstance(args, dict):
                raise Rejected("invalid_argument", "arguments must be an object")
            action = args.get("action")
            if action not in ACTIONS:
                raise Rejected("invalid_action", "unknown action")
            if action == "info":
                return self.info()
            if action == "stop":
                return self.stop()
            if action in ("stop_navigation", "pause_navigation"):
                with self.lock:
                    runtime = self.runtime
                    self.epoch += 1  # cancels any concurrent semantic request
                if runtime is None:
                    return {"status": "stopped", "terminal_confirmed": True}
                if action == "pause_navigation":
                    runtime.pause()
                    return {"status": "paused"}
                if not runtime.stop_navigation():
                    raise Rejected("stop_unconfirmed", "runtime or physical stop not confirmed")
                return {"status": "stopped", "terminal_confirmed": True}
            if not self.operations.acquire(blocking=False):
                raise Rejected("operation_busy", "another operation is running; stop remains available")
            try:
                return self._action(action, args)
            finally:
                self.operations.release()
        except Rejected as exc:
            self.last_error = exc.code
            return {"status": "error", "state": "error", "error_code": exc.code,
                    "error": str(exc), "terminal_confirmed": False}
        except Exception as exc:
            self.last_error = type(exc).__name__
            return {"status": "error", "state": "error", "error_code": "runtime_failed",
                    "error": type(exc).__name__, "terminal_confirmed": False}

    def _action(self, action, args):
        if action == "config":
            with self.lock:
                if self.runtime or self.starting:
                    raise Rejected("config_requires_stop", "stop card before changing config")
                changes = {k: v for k, v in args.items() if k not in ("action", "instance_id")}
                if changes.get("vlm_api_key") == "****":
                    changes.pop("vlm_api_key")
                self.cfg = config({**self.cfg, **changes})
                self.store = None
            return self.info()
        if action == "start":
            return self._start(args)
        if action == "list_maps":
            return {"status": "ok", "maps": self._store().maps()}
        with self.lock:
            runtime, epoch = self.runtime, self.epoch
        if runtime is None or runtime.cancelled.is_set():
            raise Rejected("not_started", "start card first")
        if runtime.gate.nav_id and action not in ("list_points", "resume_navigation"):
            raise Rejected("navigation_active", "stop active navigation first")
        if action == "start_mapping":
            if runtime.mode == "mapping":
                raise Rejected("mapping_active", "mapping already active")
            name = args.get("map_name")
            if not isinstance(name, str) or not 1 <= len(name.strip()) <= 128:
                raise Rejected("invalid_argument", "map_name is required")
            if runtime.blocker() not in ("", "localization_unconfirmed", "costmap_stale"):
                raise Rejected("sensors_not_ready", runtime.blocker())
            runtime.launch("mapping")
            self.mapping_name, self.map_item = name.strip(), None
            return {"status": "mapping", "ready": False}
        if action == "stop_mapping":
            if runtime.mode != "mapping":
                raise Rejected("not_mapping", "no active mapping session")
            map_id, prefix = self._store().allocate()
            runtime.save_map(prefix)
            item = self._store().finalize(map_id, self.mapping_name)
            runtime.end_stack()
            return {"status": "saved", "map": item}
        if action == "load_map":
            if runtime.mode == "mapping":
                raise Rejected("mapping_active", "save or stop mapping before loading")
            item = self._store().map(args.get("map_id", ""))
            runtime.launch("localization", item["yaml"])
            self.map_item = item
            return {"status": "map_loaded", "map": item, "ready": False,
                    "message": "wait for runtime, then relocalize with an initial pose"}
        if action == "relocalize":
            return runtime.relocalize(goal(args))
        if action == "resume_navigation":
            runtime.resume()
            return {"status": "navigating", "nav_id": runtime.gate.nav_id}
        item = self.map_item
        if not item:
            raise Rejected("map_not_loaded", "load a saved map first")
        self._store().map(item["map_id"])
        if action == "list_points":
            return {"status": "ok", "points": self._store().points(item["map_id"], item["revision"])}
        if action == "delete_point":
            deleted = self._store().delete_point(args.get("point_id"), item["map_id"])
            return {"status": "ok", "deleted": deleted}
        if action == "capture":
            if not self.cfg["rgb_topic"]:
                raise Rejected("rgb_not_connected", "configure and connect RGB for capture")
            pose, stamp, jpeg = runtime.capture_frame()
            description = ask(self.cfg, "Describe this location in concise Chinese for later navigation. "
                              "Use visible stable landmarks. Do not invent a location.", jpeg)
            with self.lock:
                if epoch != self.epoch or not runtime.pose_ok:
                    raise Rejected("capture_cancelled", "session changed or localization unavailable")
                point_id = self._store().capture(item, description, pose, stamp, jpeg)
            return {"status": "recorded", "point_id": point_id, "description": description,
                    "pose": pose, "map_id": item["map_id"], "revision": item["revision"]}
        if action in ("navigate_to_pose", "navigate"):
            pose = goal(args) if action == "navigate_to_pose" else choose(
                self.cfg, args.get("query"),
                self._store().points(item["map_id"], item["revision"]))["pose"]
            with self.lock:
                if epoch != self.epoch:
                    raise Rejected("cancelled", "navigation request cancelled")
                nav_id = uuid.uuid4().hex
                runtime.navigate(nav_id, pose)
            return {"status": "navigating", "action_id": nav_id, "nav_id": nav_id,
                    "target_pose": pose, "execution_enabled": self.cfg["execution_enabled"],
                    "shadow_only": True, "physical_execution": False}
        raise Rejected("invalid_action", "unsupported action")
