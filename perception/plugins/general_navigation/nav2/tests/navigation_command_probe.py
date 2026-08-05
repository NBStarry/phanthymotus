#!/usr/bin/env python3
"""Exercise the JSON command bridge against the isolated Nav2 action server."""

from __future__ import annotations

import json
import math
import os
import time
import uuid

import rclpy
from g1_nav2.execution_protocol import ProtocolError, VelocityProposal
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


COMMAND_TOPIC = "/ubuntu/navigation/nav2/command"
STATUS_TOPIC = "/ubuntu/navigation/nav2/status"
PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"


class NavigationCommandProbe(Node):
    def __init__(self) -> None:
        super().__init__("navigation_command_probe")
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._responses: dict[str, dict] = {}
        self._proposals: list[dict] = []
        self._publisher = self.create_publisher(String, COMMAND_TOPIC, command_qos)
        self._subscription = self.create_subscription(
            String, STATUS_TOPIC, self._on_status, status_qos
        )
        self._proposal_subscription = self.create_subscription(
            String, PROPOSAL_TOPIC, self._on_proposal, command_qos
        )

    def wait_for_bridge(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while self._publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"no command subscriber on {COMMAND_TOPIC}")
            rclpy.spin_once(self, timeout_sec=0.1)

    def request(
        self,
        action: str,
        args: dict,
        *,
        nav_id: str | None = None,
        timeout_sec: float = 10.0,
    ) -> dict:
        request_id = f"smoke-{uuid.uuid4().hex}"
        message = String()
        message.data = json.dumps(
            {
                "request_id": request_id,
                "nav_id": nav_id,
                "action": action,
                "args": args,
            },
            separators=(",", ":"),
        )
        self._publisher.publish(message)

        deadline = time.monotonic() + timeout_sec
        while request_id not in self._responses:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"no bridge response for {action}")
            rclpy.spin_once(self, timeout_sec=0.1)
        response = self._responses.pop(request_id)
        if response.get("shadow_only") is not True:
            raise RuntimeError(f"{action} response is not shadow-only: {response}")
        if response.get("physical_execution") is not False:
            raise RuntimeError(f"{action} response enables physical execution: {response}")
        return response

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_proposal(self, nav_id: str, timeout_sec: float = 3.0) -> dict:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for payload in reversed(self._proposals):
                if payload.get("nav_id") == nav_id:
                    try:
                        VelocityProposal.from_payload(payload)
                    except ProtocolError as exc:
                        raise RuntimeError(
                            f"unsafe N5 proposal {exc.code}: {exc}: {payload}"
                        ) from exc
                    return payload
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"no structured velocity proposal for {nav_id}")

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict) or payload.get("event") != "response":
            return
        request_id = payload.get("request_id")
        if isinstance(request_id, str) and request_id:
            self._responses[request_id] = payload

    def _on_proposal(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self._proposals.append(payload)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _tag_with_retry(node: NavigationCommandProbe, map_name: str) -> dict:
    deadline = time.monotonic() + 12.0
    last_response = None
    while time.monotonic() < deadline:
        last_response = node.request(
            "tag_place",
            {"name": "origin", "description": "synthetic mapping origin"},
        )
        if last_response.get("status") == "tagged":
            return last_response
        if last_response.get("error_code") != "map_pose_unavailable":
            break
        node.spin_for(0.5)
    raise RuntimeError(f"could not tag {map_name}: {last_response}")


def _exercise_shadow_navigation(node: NavigationCommandProbe, action: str, args: dict) -> dict:
    nav_id = f"smoke-{action}-{uuid.uuid4().hex}"
    goal_response = node.request(action, args, nav_id=nav_id)
    _require(
        goal_response.get("status") == "navigating",
        f"Nav2 goal was not accepted: {goal_response}",
    )
    _require(
        math.isclose(
            float(goal_response.get("effective_speed_limit", -1.0)),
            0.15,
            abs_tol=1e-9,
        ),
        f"shadow speed cap was not reported: {goal_response}",
    )
    node.spin_for(0.5)
    proposal = node.wait_for_proposal(nav_id)
    _require(
        proposal.get("shadow_only") is True
        and proposal.get("physical_execution") is False,
        f"N5 proposal crossed the physical boundary: {proposal}",
    )
    stop_response = node.request("stop_nav", {}, nav_id=nav_id)
    _require(
        stop_response.get("status") == "stopped"
        or (
            stop_response.get("status") == "arrived"
            and stop_response.get("already_terminal") == "arrived"
        ),
        f"Nav2 goal did not stop cleanly: {stop_response}",
    )
    goal_response["proposal_sequence"] = proposal["sequence"]
    goal_response["proposal_ttl_ms"] = proposal["ttl_ms"]
    return goal_response


def _mapping_phase(node: NavigationCommandProbe, map_name: str) -> dict:
    initial = node.request("list_maps", {})
    _require(initial.get("status") == "ready", f"list_maps failed: {initial}")
    _require(initial.get("runtime_mode") == "mapping", f"wrong mode: {initial}")
    _require(initial.get("map_count") == 0, f"map store is not empty: {initial}")

    started = node.request("start_mapping", {"map_name": map_name})
    _require(started.get("status") == "mapping", f"mapping did not start: {started}")
    tagged = _tag_with_retry(node, map_name)
    tags = node.request("list_tags", {})
    _require(tags.get("tag_count") == 1, f"tag was not persisted: {tags}")

    saved = node.request("stop_mapping", {}, timeout_sec=30.0)
    _require(saved.get("status") == "saved", f"map was not saved: {saved}")
    _require(
        saved.get("mode_switch_required") is True,
        f"mapping did not request automatic localization switch: {saved}",
    )
    listed = node.request("list_maps", {})
    _require(listed.get("map_count") == 1, f"saved map is missing: {listed}")
    _require(
        listed["maps"][0].get("map_name") == map_name,
        f"wrong saved map: {listed}",
    )
    wrong_mode = node.request("load_map", {"map_name": map_name})
    _require(
        wrong_mode.get("status") == "error"
        and wrong_mode.get("error_code") == "runtime_mode_mismatch",
        f"mapping runtime did not reject load_map: {wrong_mode}",
    )
    goal = _exercise_shadow_navigation(
        node,
        "navigate_to_tag",
        {"tag_name": "origin", "speed": 0.2, "mode": 0},
    )
    return {
        "phase": "mapping",
        "mapping_status": started["status"],
        "tag_status": tagged["status"],
        "save_status": saved["status"],
        "saved_maps": listed["map_count"],
        "mode_gate": wrong_mode["error_code"],
        "goal_status": goal["status"],
        "effective_speed_limit": goal["effective_speed_limit"],
    }


def _localization_phase(node: NavigationCommandProbe, map_name: str) -> dict:
    listed = node.request("list_maps", {})
    _require(listed.get("runtime_mode") == "localization", f"wrong mode: {listed}")
    _require(listed.get("map_count") == 1, f"saved map is missing: {listed}")
    tags = node.request("list_tags", {})
    _require(tags.get("tag_count") == 1, f"saved tag is missing: {tags}")

    loaded = node.request("load_map", {"map_name": map_name})
    _require(loaded.get("status") == "localized", f"map did not load: {loaded}")
    node.spin_for(1.0)
    wrong_mode = node.request("start_mapping", {"map_name": "forbidden"})
    _require(
        wrong_mode.get("status") == "switching"
        and wrong_mode.get("next_runtime_mode") == "mapping"
        and wrong_mode.get("retry_action_after_switch") is True,
        f"localization runtime did not request mapping switch: {wrong_mode}",
    )
    goal = _exercise_shadow_navigation(
        node,
        "navigate_to_tag",
        {"tag_name": "origin", "speed": 0.2, "mode": 0},
    )
    return {
        "phase": "localization",
        "load_status": loaded["status"],
        "saved_maps": listed["map_count"],
        "saved_tags": tags["tag_count"],
        "mode_switch": wrong_mode["next_runtime_mode"],
        "goal_status": goal["status"],
        "effective_speed_limit": goal["effective_speed_limit"],
    }


def main() -> None:
    phase = os.environ.get("N3_PROBE_PHASE", "mapping")
    map_name = os.environ.get("NAV2_SMOKE_MAP_NAME", "smoke-map")
    rclpy.init()
    node = NavigationCommandProbe()
    try:
        node.wait_for_bridge()
        if phase == "mapping":
            result = _mapping_phase(node, map_name)
        elif phase == "localization":
            result = _localization_phase(node, map_name)
        else:
            raise RuntimeError(f"unsupported N3_PROBE_PHASE: {phase}")

        result.update({"shadow_only": True, "physical_execution": False})
        print(json.dumps(result, separators=(",", ":")))
        print(f"G1_NAV2_N3_{phase.upper()}=PASS")
        print("G1_NAV2_COMMAND_BRIDGE=PASS")
        print("G1_NAV2_N5_PROPOSAL=PASS")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
