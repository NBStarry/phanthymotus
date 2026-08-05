#!/usr/bin/env python3
"""Send one isolated Nav2 goal, observe shadow outputs, then cancel it."""

from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
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
from tf2_ros import Buffer, TransformListener


SHADOW_TOPIC = "/ubuntu/navigation/nav2/cmd_vel_shadow"
RAW_TOPIC = "/ubuntu/navigation/nav2/cmd_vel_raw"
PLAN_TOPIC = "/plan"
ACTION_NAME = "/navigate_to_pose"
COSTMAP_TOPIC = "/global_costmap/costmap"


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
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


def score_candidate(
    grid: OccupancyGrid,
    start_x: float,
    start_y: float,
    target_x: float,
    target_y: float,
) -> tuple[int, int, float]:
    heading = math.atan2(target_y - start_y, target_x - start_x)
    perpendicular_x = -math.sin(heading)
    perpendicular_y = math.cos(heading)
    costs: list[int] = []
    for step in range(1, 9):
        fraction = step / 8.0
        center_x = start_x + (target_x - start_x) * fraction
        center_y = start_y + (target_y - start_y) * fraction
        for lateral_offset in (-0.20, 0.0, 0.20):
            costs.append(
                occupancy_cost(
                    grid,
                    center_x + perpendicular_x * lateral_offset,
                    center_y + perpendicular_y * lateral_offset,
                )
            )
    unknown = sum(cost == 255 for cost in costs)
    known = [cost for cost in costs if cost != 255]
    maximum = max(known) if known else 255
    average = sum(known) / len(known) if known else 255.0
    return unknown, maximum, average


class ShadowGoalProbe(Node):
    def __init__(self) -> None:
        super().__init__("g1_nav2_shadow_goal_probe")
        self.costmap: OccupancyGrid | None = None
        self.plan_poses = 0
        self.raw_messages = 0
        self.shadow_messages = 0
        self.raw_peak_linear = 0.0
        self.raw_peak_angular = 0.0
        self.shadow_peak_linear = 0.0
        self.shadow_peak_angular = 0.0

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, COSTMAP_TOPIC, self._on_costmap, map_qos
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(
            self, NavigateToPose, ACTION_NAME
        )

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def enable_observers(self) -> None:
        self.create_subscription(Path, PLAN_TOPIC, self._on_plan, 10)
        self.create_subscription(Twist, RAW_TOPIC, self._on_raw, 20)
        self.create_subscription(Twist, SHADOW_TOPIC, self._on_shadow, 20)

    def _on_plan(self, message: Path) -> None:
        self.plan_poses = max(self.plan_poses, len(message.poses))

    def _on_raw(self, message: Twist) -> None:
        self.raw_messages += 1
        self.raw_peak_linear = max(
            self.raw_peak_linear,
            math.hypot(message.linear.x, message.linear.y),
        )
        self.raw_peak_angular = max(
            self.raw_peak_angular, abs(message.angular.z)
        )

    def _on_shadow(self, message: Twist) -> None:
        self.shadow_messages += 1
        self.shadow_peak_linear = max(
            self.shadow_peak_linear,
            math.hypot(message.linear.x, message.linear.y),
        )
        self.shadow_peak_angular = max(
            self.shadow_peak_angular, abs(message.angular.z)
        )


def wait_for_inputs(node: ShadowGoalProbe, timeout_sec: float = 12.0):
    deadline = time.monotonic() + timeout_sec
    transform = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if transform is None:
            try:
                transform = node.tf_buffer.lookup_transform(
                    "map", "base_link", Time()
                )
            except Exception:
                transform = None
        if node.costmap is not None and transform is not None:
            return node.costmap, transform
    raise RuntimeError("timed out waiting for global costmap and map->base_link")


def select_goal(
    grid: OccupancyGrid, transform, distance: float
) -> tuple[float, float, float]:
    start_x = transform.transform.translation.x
    start_y = transform.transform.translation.y
    yaw = quaternion_yaw(transform.transform.rotation)
    relative_angles = (0, 45, -45, 90, -90, 135, -135, 180)
    candidates = []
    for relative_degrees in relative_angles:
        angle = yaw + math.radians(relative_degrees)
        target_x = start_x + distance * math.cos(angle)
        target_y = start_y + distance * math.sin(angle)
        unknown, maximum, average = score_candidate(
            grid, start_x, start_y, target_x, target_y
        )
        print(
            "G1_NAV2_GOAL_CANDIDATE="
            f"relative_deg:{relative_degrees},x:{target_x:.3f},"
            f"y:{target_y:.3f},unknown:{unknown},"
            f"max_cost:{maximum},mean_cost:{average:.1f}"
        )
        candidates.append(
            (
                unknown,
                maximum,
                average,
                relative_angles.index(relative_degrees),
                target_x,
                target_y,
            )
        )
    candidates.sort()
    unknown, maximum, _average, _order, target_x, target_y = candidates[0]
    if unknown != 0 or maximum >= 90:
        raise RuntimeError(
            "no sufficiently clear 0.6 m candidate in the live global costmap"
        )
    return target_x, target_y, yaw


def cancel_goal(node: ShadowGoalProbe, goal_handle) -> bool:
    cancel_future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=6.0)
    if not cancel_future.done() or cancel_future.result() is None:
        return False
    return bool(cancel_future.result().goals_canceling)


def run() -> int:
    dry_run = os.environ.get("G1_NAV2_GOAL_DRY_RUN", "0") == "1"
    distance = float(os.environ.get("G1_NAV2_GOAL_DISTANCE_M", "0.6"))
    observe_sec = float(
        os.environ.get("G1_NAV2_GOAL_OBSERVE_SEC", "8.0")
    )
    if not 0.4 <= distance <= 1.0:
        raise RuntimeError("goal distance must be within [0.4, 1.0] meters")
    if not 3.0 <= observe_sec <= 12.0:
        raise RuntimeError("observation window must be within [3, 12] seconds")

    rclpy.init()
    node = ShadowGoalProbe()
    goal_handle = None
    cancelled = False
    try:
        graph_deadline = time.monotonic() + 6.0
        required_topics = {RAW_TOPIC, SHADOW_TOPIC, PLAN_TOPIC}
        topic_names: set[str] = set()
        while time.monotonic() < graph_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            topic_names = {
                name for name, _types in node.get_topic_names_and_types()
            }
            if required_topics.issubset(topic_names):
                break
        if "/cmd_vel" in topic_names:
            raise RuntimeError("root /cmd_vel exists; refusing shadow goal")
        missing = sorted(required_topics - topic_names)
        if missing:
            raise RuntimeError(f"required topics are absent: {missing}")

        shadow_subscribers = node.get_subscriptions_info_by_topic(SHADOW_TOPIC)
        twist_shadow_subscribers = [
            endpoint
            for endpoint in shadow_subscribers
            if not getattr(endpoint, "topic_type", "")
            or endpoint.topic_type == "geometry_msgs/msg/Twist"
        ]
        external_shadow_subscribers = [
            endpoint
            for endpoint in twist_shadow_subscribers
            if endpoint.node_name != "g1_nav2_navigation_command"
        ]
        if external_shadow_subscribers:
            names = sorted(
                f"{endpoint.node_namespace}/{endpoint.node_name}"
                for endpoint in external_shadow_subscribers
            )
            raise RuntimeError(
                "shadow output already has subscribers; refusing goal: "
                + ",".join(names)
            )
        print("G1_NAV2_SHADOW_EXTERNAL_SUBSCRIBERS=0")
        print(
            "G1_NAV2_SHADOW_INTERNAL_WRAPPERS="
            f"{len(twist_shadow_subscribers) - len(external_shadow_subscribers)}"
        )
        print(
            "G1_NAV2_SHADOW_NON_TWIST_OBSERVERS="
            f"{len(shadow_subscribers) - len(twist_shadow_subscribers)}"
        )
        print("G1_NAV2_ROOT_CMD_VEL=absent")

        grid, transform = wait_for_inputs(node)
        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y
        current_yaw = quaternion_yaw(transform.transform.rotation)
        print(
            "G1_NAV2_CURRENT_POSE="
            f"x:{current_x:.3f},y:{current_y:.3f},yaw:{current_yaw:.3f}"
        )
        target_x, target_y, target_yaw = select_goal(
            grid, transform, distance
        )
        print(
            "G1_NAV2_SELECTED_GOAL="
            f"x:{target_x:.3f},y:{target_y:.3f},yaw:{target_yaw:.3f}"
        )

        if not node.action_client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("/navigate_to_pose action server is unavailable")
        if dry_run:
            print("G1_NAV2_SHADOW_GOAL_DRY_RUN=PASS")
            return 0

        node.enable_observers()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = target_x
        goal.pose.pose.position.y = target_y
        goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        send_future = node.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError("NavigateToPose goal request timed out")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")
        print("G1_NAV2_SHADOW_GOAL_ACCEPTED=yes")

        result_future = goal_handle.get_result_async()
        observation_deadline = time.monotonic() + observe_sec
        while time.monotonic() < observation_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if result_future.done():
                break
            raw_nonzero = max(
                node.raw_peak_linear, node.raw_peak_angular
            ) > 0.001
            shadow_nonzero = max(
                node.shadow_peak_linear, node.shadow_peak_angular
            ) > 0.001
            if node.plan_poses >= 2 and raw_nonzero and shadow_nonzero:
                break

        cancelled = cancel_goal(node, goal_handle)
        print(f"G1_NAV2_SHADOW_GOAL_CANCELLED={'yes' if cancelled else 'no'}")
        print(f"G1_NAV2_PLAN_POSES={node.plan_poses}")
        print(
            "G1_NAV2_RAW_CMD="
            f"messages:{node.raw_messages},"
            f"peak_linear:{node.raw_peak_linear:.4f},"
            f"peak_angular:{node.raw_peak_angular:.4f}"
        )
        print(
            "G1_NAV2_SHADOW_CMD="
            f"messages:{node.shadow_messages},"
            f"peak_linear:{node.shadow_peak_linear:.4f},"
            f"peak_angular:{node.shadow_peak_angular:.4f}"
        )

        raw_nonzero = max(node.raw_peak_linear, node.raw_peak_angular) > 0.001
        shadow_nonzero = max(
            node.shadow_peak_linear, node.shadow_peak_angular
        ) > 0.001
        if node.plan_poses < 2:
            raise RuntimeError("no non-empty Nav2 plan was observed")
        if not raw_nonzero:
            raise RuntimeError("no non-zero raw cmd_vel was observed")
        if not shadow_nonzero:
            raise RuntimeError("no non-zero shadow cmd_vel was observed")
        if not cancelled:
            raise RuntimeError("goal cancellation was not acknowledged")

        print("G1_NAV2_SHADOW_GOAL_TEST=PASS")
        print("NOTE=goal was cancelled; no Driver executor was connected")
        return 0
    finally:
        if goal_handle is not None and not cancelled:
            try:
                cancel_goal(node, goal_handle)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    try:
        return run()
    except Exception as exc:
        print(f"ERROR={exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
