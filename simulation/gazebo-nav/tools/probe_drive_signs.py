#!/usr/bin/env python3
"""Assert positive drive signs, then restore the Gazebo rig to its start pose."""

import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def command_and_measure(node, publisher, samples, field, value, seconds):
    message = Twist()
    if field == "linear.x":
        message.linear.x = value
    elif field == "angular.z":
        message.angular.z = value
    else:
        raise ValueError(f"unsupported command field: {field}")
    start = len(samples)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.04)
    moving = [sample[field] for sample in samples[start:] if abs(sample[field]) >= 0.08]
    if not moving:
        raise RuntimeError(f"no moving odometry samples for {field}: {samples[start:][-20:]}")
    return statistics.median(moving[-10:]), len(moving)


def publish_stop(node, publisher):
    for _ in range(10):
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.04)


def wait_for_pose(node, samples, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not samples:
        raise RuntimeError("no odometry pose available")
    return dict(samples[-1])


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def forward_displacement(reference, current):
    dx = current["x"] - reference["x"]
    dy = current["y"] - reference["y"]
    return dx * math.cos(reference["yaw"]) + dy * math.sin(reference["yaw"])


def restore_linear_position(node, publisher, samples, reference, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = samples[-1]
        dx = current["x"] - reference["x"]
        dy = current["y"] - reference["y"]
        displacement = dx * math.cos(reference["yaw"]) + dy * math.sin(reference["yaw"])
        if abs(displacement) < 0.025:
            publish_stop(node, publisher)
            return
        message = Twist()
        message.linear.x = clamp(-1.2 * displacement, -0.16, 0.16)
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.04)
    raise RuntimeError(f"linear pose restoration timeout: reference={reference}, current={samples[-1]}")


def restore_yaw(node, publisher, samples, reference, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        error = wrap_angle(reference["yaw"] - samples[-1]["yaw"])
        if abs(error) < 0.025:
            publish_stop(node, publisher)
            return
        message = Twist()
        message.angular.z = clamp(1.5 * error, -0.3, 0.3)
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.04)
    raise RuntimeError(f"yaw restoration timeout: reference={reference}, current={samples[-1]}")


def main():
    rclpy.init()
    node = Node("p3_cmd_odom_sign_probe")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    samples = []
    ground_truth_samples = []
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    def append_odometry(message):
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        samples.append(
            {
                "linear.x": message.twist.twist.linear.x,
                "angular.z": message.twist.twist.angular.z,
                "x": message.pose.pose.position.x,
                "y": message.pose.pose.position.y,
                "yaw": yaw,
            }
        )

    def append_ground_truth(message):
        for transform in message.transforms:
            if transform.child_frame_id != "planar_base":
                continue
            orientation = transform.transform.rotation
            ground_truth_samples.append({
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "yaw": math.atan2(
                    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
                ),
            })
            return

    node.create_subscription(
        Odometry,
        "/odom",
        append_odometry,
        qos,
    )
    node.create_subscription(
        TFMessage,
        "/world/synthetic_room/dynamic_pose/info",
        append_ground_truth,
        qos,
    )
    try:
        reference = wait_for_pose(node, samples)
        ground_truth_reference = wait_for_pose(node, ground_truth_samples)
        linear, linear_count = command_and_measure(
            node, publisher, samples, "linear.x", 0.2, 1.0
        )
        publish_stop(node, publisher)
        linear_ground_truth = forward_displacement(ground_truth_reference, ground_truth_samples[-1])
        restore_linear_position(node, publisher, samples, reference)
        angular_ground_truth_reference = dict(ground_truth_samples[-1])
        angular, angular_count = command_and_measure(
            node, publisher, samples, "angular.z", 0.4, 1.0
        )
        publish_stop(node, publisher)
        angular_ground_truth = wrap_angle(
            ground_truth_samples[-1]["yaw"] - angular_ground_truth_reference["yaw"]
        )
        restore_yaw(node, publisher, samples, reference)
        if linear <= 0.08 or angular <= 0.08 or linear_ground_truth <= 0.08 or angular_ground_truth <= 0.08:
            raise RuntimeError(
                "cmd_vel sign mismatch: "
                f"odom_linear={linear}, odom_angular={angular}, "
                f"ground_truth_linear={linear_ground_truth}, ground_truth_angular={angular_ground_truth}"
            )
        print(
            "P3 CMD_VEL PHYSICAL SIGN PASS",
            {
                "linear_command": 0.2,
                "linear_odom_median": round(linear, 4),
                "linear_ground_truth": round(linear_ground_truth, 4),
                "linear_samples": linear_count,
                "angular_command": 0.4,
                "angular_odom_median": round(angular, 4),
                "angular_ground_truth": round(angular_ground_truth, 4),
                "angular_samples": angular_count,
            },
        )
    finally:
        publish_stop(node, publisher)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
