#!/usr/bin/env python3
"""Publish deterministic G1-shaped inputs for an isolated Nav2 smoke test."""

import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String, UInt8MultiArray


class SyntheticG1Inputs(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_g1_inputs")
        self._loco = self.create_publisher(
            String, "/ubuntu/loco/state", qos_profile_sensor_data
        )
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cloud = self.create_publisher(
            UInt8MultiArray,
            "/ubuntu/lidar/cloud",
            cloud_qos,
        )
        point_count = 720
        points = b"".join(
            struct.pack(
                "<fff",
                5.0 * math.cos(math.tau * index / point_count),
                5.0 * math.sin(math.tau * index / point_count),
                0.0,
            )
            for index in range(point_count)
        )
        self._points = points
        self._point_count = point_count
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        loco = String()
        loco.data = json.dumps(
            {
                "schema_version": 2,
                "source_stamp_ns": time.time_ns(),
                "frame_id": "odom_source",
                "position": [1.0, 2.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
                "yaw_speed": 0.0,
                "imu": {
                    "rpy": [0.0, 0.0, 0.25],
                    "quaternion": [1.0, 0.0, 0.0, 0.0],
                },
            }
        )
        self._loco.publish(loco)

        cloud = UInt8MultiArray()
        frame_id = b"livox_frame"
        envelope = struct.pack(
            "<4sHHIIqH",
            b"PCV2",
            2,
            0,
            12,
            self._point_count,
            time.time_ns(),
            len(frame_id),
        )
        cloud.data = list(envelope + frame_id + self._points)
        self._cloud.publish(cloud)


def main() -> None:
    rclpy.init()
    node = SyntheticG1Inputs()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
