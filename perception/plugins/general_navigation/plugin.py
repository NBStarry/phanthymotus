"""Perception Bundle adapter for the General Navigation card."""

from __future__ import annotations

import logging
import threading

from .backend import RosTopicNavigationBackend
from .contract import general_navigation_tool_definition
from .core import GeneralNavigationCore, UnavailableNavigationBackend


log = logging.getLogger(__name__)


class GeneralNavigationPlugin:
    """Expose the frozen 14-action navigation contract as `general_navigation`."""

    PREFIX = "general"

    def __init__(
        self,
        plugin_cfg: dict,
        namespace: str,
        executor,
        *,
        backend=None,
    ):
        self._cfg = plugin_cfg
        self._namespace = namespace.strip("/")
        if backend is None:
            backend = self._build_backend(plugin_cfg, executor)
        self._core = GeneralNavigationCore(backend)
        self._lifecycle_lock = threading.RLock()
        self._canvas_started = False
        self._canvas_instance_id = ""
        self._wired_topics: dict[str, str] = {}

    def get_tools(self) -> list:
        return [general_navigation_tool_definition(self._namespace)]

    def dispatch(self, name: str, args: dict) -> dict | None:
        if name != "navigation":
            return None
        if not isinstance(args, dict):
            return self._error("invalid_argument", "arguments must be an object")
        action = args.get("action")
        if action == "start":
            return self._start_canvas(args)
        if action == "stop":
            return self._stop_canvas()
        if action == "info":
            return self._info()
        with self._lifecycle_lock:
            canvas_started = self._canvas_started
        if not canvas_started:
            return self._error(
                "canvas_not_started",
                "connect loco_state and lidar_cloud, then start the canvas project",
            )
        return self._core.dispatch(args)

    def stop(self) -> None:
        self._stop_canvas()
        self._core.stop()

    @staticmethod
    def _error(code: str, message: str) -> dict:
        return {
            "state": "error",
            "status": "error",
            "error_code": code,
            "error": message,
            "message": message,
            "shadow_only": True,
            "physical_execution": False,
        }

    def _start_canvas(self, args: dict) -> dict:
        tool = self.get_tools()[0]
        expected = {
            item["port"]: item["topic"]
            for item in tool["topic_in"]
        }
        required_ports = {
            item["port"]
            for item in tool["topic_in"]
            if item.get("required", True)
        }
        raw_topics = args.get("input_topics", [])
        if isinstance(raw_topics, str):
            raw_topics = [raw_topics]
        if not isinstance(raw_topics, list) or any(
            not isinstance(topic, str) for topic in raw_topics
        ):
            return self._error(
                "invalid_canvas_wiring", "input_topics must be an array of topic names"
            )
        single_topic = args.get("input_topic")
        if single_topic:
            if not isinstance(single_topic, str):
                return self._error(
                    "invalid_canvas_wiring", "input_topic must be a topic name"
                )
            raw_topics = [*raw_topics, single_topic]

        unique_topics = {topic.strip() for topic in raw_topics if topic.strip()}
        raw_bindings = args.get("input_bindings", [])
        if raw_bindings is None:
            raw_bindings = []
        if not isinstance(raw_bindings, list) or any(
            not isinstance(binding, dict) for binding in raw_bindings
        ):
            return self._error(
                "invalid_canvas_wiring", "input_bindings must be an array"
            )
        binding_ports = [str(binding.get("port", "")) for binding in raw_bindings]
        duplicate_ports = sorted(
            port for port in set(binding_ports) if binding_ports.count(port) > 1
        )
        if duplicate_ports:
            return self._error(
                "invalid_canvas_wiring",
                "input_bindings contain duplicate ports: "
                + ",".join(duplicate_ports),
            )
        bound_topics = {
            str(binding.get("port", "")): str(binding.get("topic", "")).strip()
            for binding in raw_bindings
            if str(binding.get("port", "")) in expected
            and str(binding.get("topic", "")).strip()
        }
        if raw_bindings:
            unknown_ports = sorted(
                str(binding.get("port", ""))
                for binding in raw_bindings
                if str(binding.get("port", "")) not in expected
            )
            missing_ports = sorted(required_ports - set(bound_topics))
            wrong_required = sorted(
                port
                for port in required_ports
                if bound_topics.get(port) and bound_topics[port] != expected[port]
            )
            missing = missing_ports + wrong_required
            unexpected = unknown_ports
        else:
            required_topics = {expected[port] for port in required_ports}
            optional_count = len(expected) - len(required_ports)
            missing = sorted(required_topics - unique_topics)
            unexpected_topics = unique_topics - required_topics
            unexpected = (
                sorted(unexpected_topics)
                if len(unexpected_topics) > optional_count
                else []
            )
        if missing or unexpected:
            details = []
            if missing:
                details.append("missing=" + ",".join(missing))
            if unexpected:
                details.append("unexpected=" + ",".join(unexpected))
            return self._error(
                "invalid_canvas_wiring",
                "general navigation requires its two Driver sensor inputs ("
                + "; ".join(details)
                + ")",
            )

        backend_info = self._core.info()
        backend_state = str(backend_info.get("state", "idle"))
        if backend_state in {
            "unavailable",
            "error",
            "waiting_for_nav2_companion",
        }:
            reason = str(
                backend_info.get("reason")
                or backend_info.get("error")
                or backend_state
            )
            return self._error("backend_not_ready", reason)
        if backend_info.get("backend") == "nav2_ros_topic":
            if int(backend_info.get("bridge_subscribers", 0)) < 1:
                return self._error(
                    "nav2_companion_unavailable",
                    "Nav2 companion is not subscribed to the command topic",
                )
            if backend_info.get("n3_ready") is not True:
                blockers = backend_info.get("readiness_blockers") or [
                    "Nav2 companion has not published a ready runtime receipt"
                ]
                return self._error("navigation_not_ready", "; ".join(blockers))

        instance_id = str(args.get("instance_id", "") or "default").strip()
        wired_topics = dict(bound_topics)
        if not wired_topics:
            wired_topics = {
                port: topic for port, topic in expected.items() if topic in unique_topics
            }
        with self._lifecycle_lock:
            self._canvas_started = True
            self._canvas_instance_id = instance_id
            self._wired_topics = wired_topics
        return {
            "state": "ready",
            "canvas_wired": True,
            "instance_id": instance_id,
            "topic_in": [
                {
                    **item,
                    "connected": (
                        wired_topics.get(item["port"]) == item["topic"]
                        if item.get("required", True)
                        else bool(wired_topics.get(item["port"]))
                    ),
                }
                for item in tool["topic_in"]
            ],
            "topic_out": tool["topic_out"],
            "shadow_only": True,
            "physical_execution": False,
        }

    def _stop_canvas(self) -> dict:
        with self._lifecycle_lock:
            was_started = self._canvas_started
            self._canvas_started = False
            self._canvas_instance_id = ""
            self._wired_topics = {}
        stop_result = None
        if was_started and self._core.info().get("active_nav_id"):
            stop_result = self._core.dispatch({"action": "stop_nav"})
        return {
            "state": "idle",
            "canvas_wired": False,
            "stop_result": stop_result,
            "shadow_only": True,
            "physical_execution": False,
        }

    def _build_backend(self, cfg: dict, executor):
        if cfg.get("shadow_only", True) is not True:
            return UnavailableNavigationBackend(
                "Perception navigation refuses non-shadow configuration"
            )
        backend_name = str(cfg.get("backend", "ros_topic")).strip().lower()
        if backend_name in {"disabled", "none"}:
            return UnavailableNavigationBackend("navigation backend is disabled")
        if backend_name != "ros_topic":
            return UnavailableNavigationBackend(
                f"unsupported navigation backend: {backend_name}"
            )
        try:
            return RosTopicNavigationBackend(cfg, self._namespace, executor)
        except Exception as exc:
            log.error("[general_navigation] backend unavailable: %s", exc, exc_info=True)
            return UnavailableNavigationBackend(
                f"Nav2 ROS topic backend unavailable: {type(exc).__name__}: {exc}"
            )

    def _info(self) -> dict:
        tool = self.get_tools()[0]
        result = self._core.info()
        with self._lifecycle_lock:
            canvas_started = self._canvas_started
            instance_id = self._canvas_instance_id
            wired_topics = dict(self._wired_topics)
        result.update(
            {
                "name": "GeneralNavigation",
                "type": "processor",
                "canvas_wired": canvas_started,
                "instance_id": instance_id or None,
                "topic_in": [
                    {
                        **item,
                        "connected": (
                            wired_topics.get(item["port"]) == item["topic"]
                            if item.get("required", True)
                            else bool(wired_topics.get(item["port"]))
                        ),
                    }
                    for item in tool["topic_in"]
                ],
                "topic_out": tool["topic_out"],
            }
        )
        return result
