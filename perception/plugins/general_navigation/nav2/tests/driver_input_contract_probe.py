#!/usr/bin/env python3
"""Read-only live probe for the released G1 Driver navigation inputs."""

from __future__ import annotations

import json
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8MultiArray


HEADER = struct.Struct("<4sHHIIqH")
LEGACY_HEADER = struct.Struct("<II")


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("g1_navigation_driver_input_contract_probe")
        self.loco = None
        self.cloud = None
        self.done = threading.Event()
        self.create_subscription(
            String,
            "/ubuntu/loco/state",
            self.on_loco,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt8MultiArray,
            "/ubuntu/lidar/cloud",
            self.on_cloud,
            qos_profile_sensor_data,
        )

    def on_loco(self, message: String) -> None:
        self.loco = json.loads(message.data)
        self.finish_if_ready()

    def on_cloud(self, message: UInt8MultiArray) -> None:
        self.cloud = bytes(message.data)
        self.finish_if_ready()

    def finish_if_ready(self) -> None:
        if self.loco is not None and self.cloud is not None:
            self.done.set()


def fresh(stamp_ns: int) -> bool:
    age_ns = time.time_ns() - stamp_ns
    return -100_000_000 <= age_ns <= 500_000_000


def finite_vector(value, size: int) -> bool:
    if not isinstance(value, list) or len(value) < size:
        return False
    try:
        import math

        return all(math.isfinite(float(item)) for item in value[:size])
    except (TypeError, ValueError):
        return False


def validate_loco(loco: dict) -> dict:
    assert isinstance(loco, dict), loco
    assert finite_vector(loco.get("position"), 3), loco
    assert finite_vector(loco.get("velocity"), 3), loco
    assert isinstance(loco.get("imu"), dict), loco
    assert finite_vector(loco["imu"].get("rpy"), 3), loco
    assert finite_vector(loco["imu"].get("quaternion"), 4), loco
    assert isinstance(loco.get("yaw_speed"), (int, float)), loco
    if loco.get("schema_version") == 2:
        assert loco.get("frame_id") == "odom_source", loco
        assert isinstance(loco.get("source_stamp_ns"), int), loco
        assert fresh(loco["source_stamp_ns"]), loco
        return {
            "schema": "phanthy.g1.loco_state.v2",
            "timestamp_source": "driver",
            "frame_source": "driver_payload",
        }
    assert "schema_version" not in loco, loco
    assert "source_stamp_ns" not in loco, loco
    assert "frame_id" not in loco, loco
    return {
        "schema": "unitree.g1.loco_state.legacy",
        "timestamp_source": "adapter_receive",
        "frame_source": "adapter_contract",
    }


def validate_cloud(raw: bytes) -> dict:
    if raw[:4] == b"PCV2":
        assert len(raw) >= HEADER.size, len(raw)
        magic, version, flags, point_step, count, stamp_ns, frame_len = HEADER.unpack_from(raw)
        assert (magic, version, flags) == (b"PCV2", 2, 0), (magic, version, flags)
        assert fresh(stamp_ns), stamp_ns
        end = HEADER.size + frame_len
        frame_id = raw[HEADER.size:end].decode("utf-8")
        assert frame_id and not frame_id.startswith("/"), frame_id
        schema = "phanthy.sensor.pointcloud.v2"
        timestamp_source = "driver"
        frame_source = "driver_payload"
    else:
        assert len(raw) >= LEGACY_HEADER.size, len(raw)
        point_step, count = LEGACY_HEADER.unpack_from(raw)
        end = LEGACY_HEADER.size
        frame_id = "livox_frame"
        schema = "unitree.g1.pointcloud.legacy"
        timestamp_source = "adapter_receive"
        frame_source = "adapter_contract"
    assert 12 <= point_step <= 512 and 0 < count <= 2_000_000, (point_step, count)
    assert len(raw) == end + point_step * count, (
        len(raw),
        end,
        point_step,
        count,
    )
    return {
        "schema": schema,
        "timestamp_source": timestamp_source,
        "frame_source": frame_source,
        "frame_id": frame_id,
        "point_step": point_step,
        "point_count": count,
    }


def main() -> None:
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 15.0
    try:
        while not node.done.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node.done.is_set(), "timed out waiting for both Driver inputs"
        loco_contract = validate_loco(node.loco)
        cloud_contract = validate_cloud(node.cloud)
        print(
            json.dumps(
                {
                    "loco": loco_contract,
                    "cloud": cloud_contract,
                },
                separators=(",", ":"),
            )
        )
        print("G1_NAVIGATION_DRIVER_INPUT_CONTRACT=PASS")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
