#!/usr/bin/env python3
"""Run one explicit G1 owner phase against the N3 JSON command bridge."""

from __future__ import annotations

import json
import os
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Empty
from std_msgs.msg import String


COMMAND_TOPIC = "/ubuntu/navigation/nav2/command"
STATUS_TOPIC = "/ubuntu/navigation/nav2/status"


class OwnerProbe(Node):
    def __init__(self) -> None:
        super().__init__("g1_nav2_n3_owner_probe")
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
        self._publisher = self.create_publisher(String, COMMAND_TOPIC, command_qos)
        self._subscription = self.create_subscription(
            String, STATUS_TOPIC, self._on_status, status_qos
        )
        self._global_localization_client = self.create_client(
            Empty, "/reinitialize_global_localization"
        )

    def wait_for_bridge(self, timeout_sec: float = 8.0) -> None:
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
        timeout_sec: float = 40.0,
    ) -> dict:
        request_id = f"owner-{uuid.uuid4().hex}"
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
            raise RuntimeError(f"{action} enabled physical execution: {response}")
        return response

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def request_global_localization(self, timeout_sec: float = 15.0) -> None:
        if not self._global_localization_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            raise RuntimeError(
                "AMCL service is unavailable: /reinitialize_global_localization"
            )
        future = self._global_localization_client.call_async(Empty.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("AMCL global localization request timed out")
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.exception() is not None:
            raise RuntimeError(
                f"AMCL global localization failed: {future.exception()}"
            )

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


def require_ok(response: dict, expected_status: str) -> None:
    if response.get("status") != expected_status:
        raise RuntimeError(f"expected {expected_status}, got {response}")


def tag_origin(node: OwnerProbe) -> dict:
    deadline = time.monotonic() + 15.0
    response = None
    while time.monotonic() < deadline:
        response = node.request(
            "tag_place",
            {"name": "origin", "description": "N3 mapping origin"},
        )
        if response.get("status") == "tagged":
            return response
        if response.get("error_code") != "map_pose_unavailable":
            break
        node.spin_for(0.5)
    raise RuntimeError(f"origin tag failed: {response}")


def begin(node: OwnerProbe, map_name: str) -> dict:
    listed = node.request("list_maps", {})
    if any(item.get("map_name") == map_name for item in listed.get("maps", [])):
        raise RuntimeError(f"map already exists: {map_name}")
    started = node.request("start_mapping", {"map_name": map_name})
    require_ok(started, "mapping")
    tagged = tag_origin(node)
    return {
        "phase": "begin",
        "map_name": map_name,
        "mapping_status": started["status"],
        "origin_tag": tagged["status"],
    }


def save(node: OwnerProbe, map_name: str) -> dict:
    tags = node.request("list_tags", {})
    if tags.get("map_name") != map_name:
        raise RuntimeError(f"active map mismatch: {tags}")
    saved = node.request("stop_mapping", {}, timeout_sec=45.0)
    require_ok(saved, "saved")
    listed = node.request("list_maps", {})
    matches = [item for item in listed.get("maps", []) if item.get("map_name") == map_name]
    if len(matches) != 1 or matches[0].get("status") != "ready":
        raise RuntimeError(f"saved map is not ready: {listed}")
    return {
        "phase": "save",
        "map_name": map_name,
        "save_status": saved["status"],
        "tag_count": matches[0].get("tag_count"),
        "mode_switch_required": saved.get("mode_switch_required"),
    }


def globalize(node: OwnerProbe, map_name: str) -> dict:
    loaded = node.request("load_map", {"map_name": map_name})
    require_ok(loaded, "localized")
    node.spin_for(1.0)
    node.request_global_localization()
    node.spin_for(1.0)
    return {
        "phase": "globalize",
        "map_name": map_name,
        "load_status": loaded["status"],
        "global_localization_status": "requested",
    }


def verify(node: OwnerProbe, map_name: str) -> dict:
    listed = node.request("list_maps", {})
    if listed.get("active_map") != map_name:
        raise RuntimeError(f"active map mismatch: {listed}")
    tags = node.request("list_tags", {})
    if not any(item.get("name") == "origin" for item in tags.get("tags", [])):
        raise RuntimeError(f"origin tag is missing: {tags}")
    node.spin_for(1.0)
    nav_id = f"n3-owner-{uuid.uuid4().hex}"
    goal = node.request(
        "navigate_to_tag",
        {"tag_name": "origin", "speed": 0.2, "mode": 0},
        nav_id=nav_id,
    )
    require_ok(goal, "navigating")
    node.spin_for(0.8)
    stopped = node.request("stop_nav", {}, nav_id=nav_id)
    require_ok(stopped, "stopped")
    return {
        "phase": "verify",
        "map_name": map_name,
        "load_status": "already_loaded",
        "tag_count": tags["tag_count"],
        "goal_status": goal["status"],
        "stop_status": stopped["status"],
        "effective_speed_limit": goal["effective_speed_limit"],
    }


def main() -> None:
    phase = os.environ["N3_OWNER_PHASE"]
    map_name = os.environ["N3_MAP_NAME"]
    rclpy.init()
    node = OwnerProbe()
    try:
        node.wait_for_bridge()
        if phase == "begin":
            result = begin(node, map_name)
        elif phase == "save":
            result = save(node, map_name)
        elif phase == "globalize":
            result = globalize(node, map_name)
        elif phase == "verify":
            result = verify(node, map_name)
        else:
            raise RuntimeError(f"unsupported owner phase: {phase}")
        result.update({"shadow_only": True, "physical_execution": False})
        print(json.dumps(result, separators=(",", ":")))
        print(f"G1_NAV2_N3_OWNER_{phase.upper()}=PASS")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
