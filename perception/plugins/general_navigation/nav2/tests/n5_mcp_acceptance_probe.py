#!/usr/bin/env python3
"""Exercise one deployed MCP goal in shadow or measured physical E2E mode."""

from __future__ import annotations

import json
import math
import os
import ssl
import time
import urllib.parse
import urllib.request
from collections.abc import Callable

import rclpy
from action_msgs.msg import GoalStatus
from g1_nav2.execution_protocol import ProtocolError, VelocityProposal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:15720/mcp")
COMMAND_BACKEND = os.environ.get("N5_COMMAND_BACKEND", "perception_mcp")
AGENT_CORE_URL = os.environ.get(
    "AGENT_CORE_URL", "https://127.0.0.1:15678/api"
).rstrip("/")
AGENT_CORE_ACCESS_TOKEN = os.environ.get("AGENT_CORE_ACCESS_TOKEN", "")
PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
COSTMAP_TOPIC = "/global_costmap/costmap"
TERMINAL_STATUSES = {
    "arrived",
    "cancelled",
    "stopped",
    "error",
    "aborted",
    "rejected",
}
MIN_GOAL_DISTANCE_M = 0.4
MAX_GOAL_DISTANCE_M = 5.0
MAX_GOAL_ENDPOINT_COST = 89


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def occupancy_cost(grid: OccupancyGrid, x: float, y: float) -> int:
    info = grid.info
    origin_yaw = quaternion_yaw(info.origin.orientation)
    delta_x = x - info.origin.position.x
    delta_y = y - info.origin.position.y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    local_x = cos_yaw * delta_x + sin_yaw * delta_y
    local_y = -sin_yaw * delta_x + cos_yaw * delta_y
    cell_x = math.floor(local_x / info.resolution)
    cell_y = math.floor(local_y / info.resolution)
    if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
        return 255
    value = int(grid.data[cell_y * info.width + cell_x])
    return 255 if value < 0 else value


def select_goal(
    grid: OccupancyGrid,
    transform,
    distance: float,
    compute_path_length: Callable[[float, float, float], float | None],
) -> tuple[float, float, float]:
    start_x = transform.transform.translation.x
    start_y = transform.transform.translation.y
    yaw = quaternion_yaw(transform.transform.rotation)
    relative_angles = (0, 45, -45, 90, -90, 135, -135, 180)
    candidates = []
    for order, relative_degrees in enumerate(relative_angles):
        angle = yaw + math.radians(relative_degrees)
        target_x = start_x + distance * math.cos(angle)
        target_y = start_y + distance * math.sin(angle)
        endpoint_cost = occupancy_cost(grid, target_x, target_y)
        path_length = None
        if endpoint_cost <= MAX_GOAL_ENDPOINT_COST:
            path_length = compute_path_length(target_x, target_y, yaw)
        print(
            "GENERAL_NAVIGATION_N5_GOAL_CANDIDATE="
            f"relative_deg:{relative_degrees},x:{target_x:.3f},"
            f"y:{target_y:.3f},endpoint_cost:{endpoint_cost},"
            f"reachable:{str(path_length is not None).lower()},"
            "path_length_m:"
            f"{'none' if path_length is None else f'{path_length:.3f}'}"
        )
        candidates.append(
            (
                path_length is None,
                float("inf") if path_length is None else path_length,
                endpoint_cost,
                order,
                target_x,
                target_y,
            )
        )
    candidates.sort()
    unreachable, _path_length, _cost, _order, target_x, target_y = candidates[0]
    if unreachable:
        raise RuntimeError(
            f"no Nav2-reachable {distance:.1f} m candidate in the live costmap"
        )
    return target_x, target_y, yaw


def rpc(method: str, params: dict | None, request_id: int) -> dict:
    body = json.dumps(
        {
            "jsonrpc": "2.0",
            "id": request_id,
            "method": method,
            "params": params or {},
        }
    ).encode()
    request = urllib.request.Request(
        MCP_URL,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=8) as response:
        payload = json.load(response)
    if "error" in payload:
        raise RuntimeError(f"{method} failed: {payload['error']}")
    return payload["result"]


def core_request(path: str, *, body: dict | None = None) -> dict:
    url = f"{AGENT_CORE_URL}/{path.lstrip('/')}"
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme != "https" or parsed.hostname not in {"127.0.0.1", "localhost"}:
        raise RuntimeError(f"refusing non-local Agent Core URL: {url!r}")
    headers = {"Content-Type": "application/json"}
    if AGENT_CORE_ACCESS_TOKEN:
        headers["Authorization"] = f"Bearer {AGENT_CORE_ACCESS_TOKEN}"
    request = urllib.request.Request(
        url,
        data=None if body is None else json.dumps(body).encode(),
        headers=headers,
        method="GET" if body is None else "POST",
    )
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(request, timeout=12, context=context) as response:
        value = json.load(response)
    if not isinstance(value, dict):
        raise RuntimeError(f"unexpected Agent Core response: {value!r}")
    return value


def framework_navigation_mcp_id() -> str:
    registry = core_request("mcp")
    matches = []
    for entry in registry.get("data", []):
        tools = entry.get("tools") or []
        if any(tool.get("name") == "general_navigation" for tool in tools):
            matches.append(entry)
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one framework general_navigation MCP, got {len(matches)}"
        )
    return str(matches[0].get("id", ""))


def require_framework_canvas() -> None:
    running = core_request("config/project-running")
    if running.get("running") is not True:
        raise RuntimeError("Agent Core project is not running")
    response = core_request("canvas/layout")
    layout = response.get("data") or {}
    cards = layout.get("cards") or []
    connections = layout.get("connections") or []

    def one(tool_name: str) -> dict:
        matches = [card for card in cards if card.get("toolName") == tool_name]
        if len(matches) != 1:
            raise RuntimeError(
                f"expected one {tool_name} canvas card, got {len(matches)}"
            )
        return matches[0]

    state = one("loco_state")
    lidar = one("lidar_cloud")
    navigation = one("general_navigation")
    loco = one("loco")

    def connected(source: dict, target: dict, topic: str) -> bool:
        return any(
            connection.get("fromCardId") == source.get("id")
            and connection.get("toCardId") == target.get("id")
            and connection.get("fromTopic") == topic
            for connection in connections
        )

    required = (
        connected(state, navigation, "/ubuntu/loco/state"),
        connected(lidar, navigation, "/ubuntu/lidar/cloud"),
        connected(navigation, loco, PROPOSAL_TOPIC),
    )
    if not all(required):
        raise RuntimeError(
            "canvas must wire loco_state + lidar_cloud -> general_navigation -> loco"
        )


def call_navigation(arguments: dict, request_id: int) -> dict:
    if COMMAND_BACKEND == "agent_core":
        result = core_request(
            f"mcp/{framework_navigation_mcp_id()}/call",
            body={"tool": "general_navigation", "arguments": arguments},
        )
        if result.get("code") != 200:
            raise RuntimeError(f"Agent Core tool call failed: {result!r}")
        content = result.get("data", [])
        if len(content) != 1 or content[0].get("type") != "text":
            raise RuntimeError(f"unexpected Agent Core tool content: {content!r}")
        payload = json.loads(content[0]["text"])
        if not isinstance(payload, dict):
            raise RuntimeError(f"tool result is not an object: {payload!r}")
        return payload
    if COMMAND_BACKEND != "perception_mcp":
        raise RuntimeError(
            "N5_COMMAND_BACKEND must be perception_mcp or agent_core"
        )
    result = rpc(
        "tools/call",
        {"name": "general_navigation", "arguments": arguments},
        request_id,
    )
    content = result.get("content", [])
    if len(content) != 1 or content[0].get("type") != "text":
        raise RuntimeError(f"unexpected tools/call content: {content!r}")
    payload = json.loads(content[0]["text"])
    if not isinstance(payload, dict):
        raise RuntimeError(f"tool result is not an object: {payload!r}")
    return payload


class ProposalProbe(Node):
    def __init__(self) -> None:
        super().__init__("g1_n5_mcp_acceptance_probe")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._proposals: list[dict] = []
        self._subscription = self.create_subscription(
            String, PROPOSAL_TOPIC, self._on_proposal, qos
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._costmap: OccupancyGrid | None = None
        self._costmap_subscription = self.create_subscription(
            OccupancyGrid, COSTMAP_TOPIC, self._on_costmap, map_qos
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._compute_path_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )

    def _on_proposal(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self._proposals.append(payload)

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self._costmap = message

    def wait_for_publisher(self, timeout_sec: float = 6.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while self.count_publishers(PROPOSAL_TOPIC) == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"no publisher on {PROPOSAL_TOPIC}")
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_navigation_inputs(self, timeout_sec: float = 12.0):
        deadline = time.monotonic() + timeout_sec
        transform = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if transform is None:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        "map", "base_link", Time()
                    )
                except Exception:
                    transform = None
            if self._costmap is not None and transform is not None:
                return self._costmap, transform
        raise RuntimeError("timed out waiting for costmap and map->base_link")

    def compute_path_length(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        timeout_sec: float = 8.0,
    ) -> float | None:
        if not self._compute_path_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("Nav2 /compute_path_to_pose action is unavailable")

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.use_start = False
        goal_future = self._compute_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout_sec)
        if not goal_future.done():
            raise RuntimeError("Nav2 path goal response timed out")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise RuntimeError("Nav2 path computation timed out")
        wrapped_result = result_future.result()
        if (
            wrapped_result is None
            or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
        ):
            return None
        poses = wrapped_result.result.path.poses
        if len(poses) < 2:
            return None
        return sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            for previous, current in zip(poses, poses[1:])
        )

    def wait_for_proposal(
        self,
        nav_id: str,
        *,
        after_sequence: int = 0,
        statuses: set[str] | None = None,
        require_nonzero: bool = False,
        timeout_sec: float = 8.0,
    ) -> VelocityProposal:
        deadline = time.monotonic() + timeout_sec
        checked = 0
        while time.monotonic() < deadline:
            while checked < len(self._proposals):
                payload = self._proposals[checked]
                checked += 1
                if payload.get("nav_id") != nav_id:
                    continue
                try:
                    proposal = VelocityProposal.from_payload(payload)
                except ProtocolError as exc:
                    raise RuntimeError(
                        f"unsafe N5 proposal {exc.code}: {exc}: {payload}"
                    ) from exc
                if proposal.sequence <= after_sequence:
                    continue
                if statuses is not None and proposal.navigation_status not in statuses:
                    continue
                if require_nonzero and proposal.velocity.is_zero():
                    continue
                return proposal
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"no matching N5 proposal for {nav_id} after sequence {after_sequence}"
        )

    def wait_for_physical_arrival(
        self,
        nav_id: str,
        *,
        target_x: float,
        target_y: float,
        after_sequence: int,
        timeout_sec: float,
    ) -> dict:
        start = self._lookup_map_pose()
        deadline = time.monotonic() + timeout_sec
        checked = 0
        last_sequence = after_sequence
        max_planar_m = 0.0
        final_pose = start
        terminal: VelocityProposal | None = None

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                final_pose = self._lookup_map_pose()
            except RuntimeError:
                pass
            max_planar_m = max(
                max_planar_m,
                math.hypot(final_pose[0] - start[0], final_pose[1] - start[1]),
            )

            while checked < len(self._proposals):
                payload = self._proposals[checked]
                checked += 1
                if payload.get("nav_id") != nav_id:
                    continue
                try:
                    proposal = VelocityProposal.from_payload(payload)
                except ProtocolError as exc:
                    raise RuntimeError(
                        f"unsafe N5 proposal {exc.code}: {exc}: {payload}"
                    ) from exc
                if proposal.sequence <= after_sequence:
                    continue
                if proposal.sequence <= last_sequence:
                    raise RuntimeError(
                        f"proposal sequence did not increase: "
                        f"{proposal.sequence} <= {last_sequence}"
                    )
                last_sequence = proposal.sequence
                if proposal.navigation_status in TERMINAL_STATUSES:
                    if not proposal.velocity.is_zero():
                        raise RuntimeError(
                            f"terminal proposal is non-zero: {proposal}"
                        )
                    terminal = proposal
                    break
            if terminal is not None:
                break

        if terminal is None:
            raise RuntimeError(
                f"navigation did not reach a terminal proposal within "
                f"{timeout_sec:.1f}s; measured_motion={max_planar_m:.3f}m"
            )
        if terminal.navigation_status != "arrived":
            raise RuntimeError(
                f"navigation terminated as {terminal.navigation_status}: {terminal}"
            )
        return {
            "terminal": terminal,
            "max_planar_m": max_planar_m,
            "final_target_distance_m": math.hypot(
                final_pose[0] - target_x, final_pose[1] - target_y
            ),
            "final_x": final_pose[0],
            "final_y": final_pose[1],
        }

    def _lookup_map_pose(self) -> tuple[float, float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_link", Time()
            )
        except Exception as exc:
            raise RuntimeError(
                f"map -> base_link transform is unavailable: {exc}"
            ) from exc
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )


def require_shadow_response(payload: dict, action: str) -> None:
    if payload.get("status") == "error":
        raise RuntimeError(f"{action} failed: {payload}")
    if payload.get("shadow_only") is not True:
        raise RuntimeError(f"{action} is not shadow-only: {payload}")
    if payload.get("physical_execution") is not False:
        raise RuntimeError(f"{action} enabled physical execution: {payload}")


def main() -> None:
    map_name = os.environ.get("N5_MAP_NAME", "g1-n3-acceptance")
    goal_distance = float(os.environ.get("N5_GOAL_DISTANCE_M", "0.6"))
    dry_run = os.environ.get("N5_DRY_RUN", "0") == "1"
    physical_e2e = os.environ.get("N5_PHYSICAL_E2E", "0") == "1"
    physical_timeout = float(os.environ.get("N5_PHYSICAL_TIMEOUT_SEC", "90"))
    min_measured_motion = float(os.environ.get("N5_MIN_MEASURED_M", "0.08"))
    arrival_tolerance = float(os.environ.get("N5_ARRIVAL_TOLERANCE_M", "0.30"))
    if not MIN_GOAL_DISTANCE_M <= goal_distance <= MAX_GOAL_DISTANCE_M:
        raise RuntimeError(
            "goal distance must be within "
            f"[{MIN_GOAL_DISTANCE_M:.1f}, {MAX_GOAL_DISTANCE_M:.1f}] meters"
        )
    if dry_run and physical_e2e:
        raise RuntimeError("N5_DRY_RUN and N5_PHYSICAL_E2E are mutually exclusive")
    if not 15.0 <= physical_timeout <= 300.0:
        raise RuntimeError("physical timeout must be within [15, 300] seconds")
    if not 0.02 <= min_measured_motion <= goal_distance:
        raise RuntimeError("minimum measured motion is outside the safe range")
    if not 0.15 <= arrival_tolerance <= 0.50:
        raise RuntimeError("arrival tolerance must be within [0.15, 0.50] meters")
    if COMMAND_BACKEND == "agent_core":
        if not AGENT_CORE_ACCESS_TOKEN:
            raise RuntimeError("AGENT_CORE_ACCESS_TOKEN is required")
        require_framework_canvas()
    else:
        rpc(
            "initialize",
            {
                "protocolVersion": "2024-11-05",
                "capabilities": {},
                "clientInfo": {"name": "g1-n5-acceptance", "version": "1"},
            },
            1,
        )

    rclpy.init()
    node = ProposalProbe()
    active_navigation = False
    request_id = 2
    try:
        node.wait_for_publisher()
        maps = call_navigation({"action": "list_maps"}, request_id)
        request_id += 1
        require_shadow_response(maps, "list_maps")
        if maps.get("runtime_mode") != "localization":
            raise RuntimeError(f"runtime is not localization: {maps}")
        if maps.get("active_map") != map_name:
            raise RuntimeError(f"active map is not {map_name}: {maps}")

        grid, transform = node.wait_for_navigation_inputs()
        target_x, target_y, target_yaw = select_goal(
            grid, transform, goal_distance, node.compute_path_length
        )
        if dry_run:
            print(
                json.dumps(
                    {
                        "map_name": map_name,
                        "goal_x": round(target_x, 3),
                        "goal_y": round(target_y, 3),
                        "goal_yaw": round(target_yaw, 3),
                        "goal_distance_m": goal_distance,
                        "shadow_only": True,
                        "physical_execution": False,
                    },
                    separators=(",", ":"),
                )
            )
            print("GENERAL_NAVIGATION_N5_GOAL_DRY_RUN=PASS")
            return

        navigating = call_navigation(
            {
                "action": "navigate_to_pose",
                "x": target_x,
                "y": target_y,
                "yaw": target_yaw,
                "speed": 0.2,
                "mode": 0,
            },
            request_id,
        )
        request_id += 1
        require_shadow_response(navigating, "navigate_to_pose")
        if navigating.get("status") != "navigating":
            raise RuntimeError(f"goal was not accepted: {navigating}")
        nav_id = navigating.get("nav_id")
        if not isinstance(nav_id, str) or not nav_id:
            raise RuntimeError(f"goal response has no nav_id: {navigating}")
        active_navigation = True

        motion = node.wait_for_proposal(
            nav_id,
            statuses={"planning", "navigating"},
            require_nonzero=True,
        )
        if motion.ttl_ms != 250:
            raise RuntimeError(f"unexpected proposal TTL: {motion.ttl_ms}")

        if physical_e2e:
            evidence = node.wait_for_physical_arrival(
                nav_id,
                target_x=target_x,
                target_y=target_y,
                after_sequence=motion.sequence,
                timeout_sec=physical_timeout,
            )
            if evidence["max_planar_m"] < min_measured_motion:
                raise RuntimeError(
                    "no measured robot motion: "
                    f"{evidence['max_planar_m']:.3f}m < {min_measured_motion:.3f}m"
                )
            if evidence["final_target_distance_m"] > arrival_tolerance:
                raise RuntimeError(
                    "robot stopped outside arrival tolerance: "
                    f"{evidence['final_target_distance_m']:.3f}m > "
                    f"{arrival_tolerance:.3f}m"
                )

            completed = call_navigation(
                {
                    "action": "wait_navigation_done",
                    "stall_timeout": physical_timeout,
                },
                request_id,
            )
            request_id += 1
            require_shadow_response(completed, "wait_navigation_done")
            if completed.get("status") != "arrived":
                raise RuntimeError(f"card did not report arrival: {completed}")
            active_navigation = False
            terminal = evidence["terminal"]
            print(
                json.dumps(
                    {
                        "map_name": map_name,
                        "goal_x": round(target_x, 3),
                        "goal_y": round(target_y, 3),
                        "goal_distance_m": goal_distance,
                        "nav_id": nav_id,
                        "proposal_sequence": motion.sequence,
                        "terminal_sequence": terminal.sequence,
                        "terminal_status": terminal.navigation_status,
                        "measured_motion_m": round(evidence["max_planar_m"], 3),
                        "final_target_distance_m": round(
                            evidence["final_target_distance_m"], 3
                        ),
                        "card_status": completed.get("status"),
                        "proposal_only": True,
                        "physical_motion_evidence": True,
                    },
                    separators=(",", ":"),
                )
            )
            print("GENERAL_NAVIGATION_LOCO_E2E_ACCEPTANCE=PASS")
            return

        stopped = call_navigation({"action": "stop_nav"}, request_id)
        request_id += 1
        require_shadow_response(stopped, "stop_nav")
        if stopped.get("status") != "stopped":
            raise RuntimeError(f"navigation did not stop: {stopped}")
        active_navigation = False

        terminal = node.wait_for_proposal(
            nav_id,
            after_sequence=motion.sequence,
            statuses=TERMINAL_STATUSES,
        )
        if not terminal.velocity.is_zero():
            raise RuntimeError(f"terminal proposal is non-zero: {terminal}")

        print(
            json.dumps(
                {
                    "map_name": map_name,
                    "goal_x": round(target_x, 3),
                    "goal_y": round(target_y, 3),
                    "goal_distance_m": goal_distance,
                    "nav_id": nav_id,
                    "goal_status": navigating["status"],
                    "proposal_sequence": motion.sequence,
                    "proposal_ttl_ms": motion.ttl_ms,
                    "terminal_sequence": terminal.sequence,
                    "terminal_status": terminal.navigation_status,
                    "stop_status": stopped["status"],
                    "shadow_only": True,
                    "physical_execution": False,
                },
                separators=(",", ":"),
            )
        )
        print("GENERAL_NAVIGATION_N5_MCP_ACCEPTANCE=PASS")
    finally:
        if active_navigation:
            try:
                call_navigation({"action": "stop_nav"}, request_id)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
