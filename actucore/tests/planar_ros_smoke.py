"""Isolated real ROS smoke: synthetic room -> SLAM -> saved map -> AMCL -> Nav2.

Run ONLY in a network-isolated container with ROS_DOMAIN_ID=191 and
ROS_LOCALHOST_ONLY=1. This is an idealized plant, NOT hardware acceptance.
"""

import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time

if os.environ.get("ROS_DOMAIN_ID") != "191" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Refusing non-isolated ROS domain: require domain 191 and localhost only")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from plugins.planar_navigation.plugin import PlanarNavigationPlugin


class Room(Node):
    def __init__(self):
        super().__init__("room_fixture")
        self.scan_pub = self.create_publisher(LaserScan, "/fixture/scan", 2)
        self.odom_pub = self.create_publisher(Odometry, "/fixture/odom", 2)
        self.create_subscription(String, "/planar_test/proposal_preview", self.command, 1)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        fixed = TransformStamped()
        fixed.header.frame_id, fixed.child_frame_id = "base_link", "laser"
        fixed.transform.rotation.w = 1.0
        self.static.sendTransform(fixed)
        self.x = self.y = self.angle = 0.0
        self.v = self.w = 0.0
        self.command_at = 0.0
        self.odom_on = self.scan_on = True
        self.max_nonzero = 0
        self.create_timer(0.02, self.tick)
        self.create_timer(0.1, self.scan)

    def command(self, msg):
        data = json.loads(msg.data)
        assert data["shadow_only"] and not data["physical_execution"]
        self.v, self.w = data["velocity"]["x"], data["velocity"]["yaw"]
        self.command_at = time.monotonic()
        self.max_nonzero += self.v != 0 or self.w != 0

    def tick(self):
        if time.monotonic() - self.command_at > 0.25:
            self.v = self.w = 0
        self.x += self.v * math.cos(self.angle) * 0.02
        self.y += self.v * math.sin(self.angle) * 0.02
        self.angle += self.w * 0.02
        if not self.odom_on:
            return
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, "odom", "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.angle/2), math.cos(self.angle/2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = self.v, self.w
        for i in (0, 7, 35):
            msg.pose.covariance[i] = 0.001
        self.odom_pub.publish(msg)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = msg.header, "base_link"
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(transform)

    def scan(self):
        if not self.scan_on:
            return
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2*math.pi/719
        msg.range_min, msg.range_max, msg.scan_time = 0.05, 15.0, 0.1
        distances = []
        for i in range(720):
            a = self.angle + msg.angle_min + i*msg.angle_increment
            dx, dy = math.cos(a), math.sin(a)
            hits = []
            for wall in (-3.0, 4.0):
                if abs(dx) > 1e-6:
                    t = (wall-self.x)/dx
                    if t > 0 and -2 <= self.y+t*dy <= 3:
                        hits.append(t)
            for wall in (-2.0, 3.0):
                if abs(dy) > 1e-6:
                    t = (wall-self.y)/dy
                    if t > 0 and -3 <= self.x+t*dx <= 4:
                        hits.append(t)
            distances.append(min(hits) if hits else float("inf"))
        msg.ranges = distances
        self.scan_pub.publish(msg)


def until(predicate, timeout, detail):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise AssertionError(detail())


def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    room = Room()
    executor.add_node(room)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    with tempfile.TemporaryDirectory(prefix="planar-ros-smoke-") as root:
        card = PlanarNavigationPlugin({
            "namespace": "planar_test", "scan_topic": "/fixture/scan",
            "odom_topic": "/fixture/odom", "data_dir": root,
            "footprint": [[-0.2,-0.15],[0.2,-0.15],[0.2,0.15],[-0.2,0.15]],
        }, executor)
        def call(action, **args):
            result = card.dispatch(card.PREFIX, {"action": action, **args})
            assert result.get("status") != "error", (action, result)
            return result
        try:
            call("start", input_topics=["/fixture/scan", "/fixture/odom"])
            until(lambda: not card.runtime.blocker(), 10, card.info)
            call("start_mapping", map_name="synthetic_room")
            until(lambda: card.runtime.grid is not None and any(v > 0 for v in card.runtime.grid.data),
                  40, card.info)
            saved = call("stop_mapping")["map"]
            call("load_map", map_id=saved["map_id"])
            until(lambda: card.runtime.states.get("amcl", (0,))[0] == 3, 40, card.info)
            call("relocalize", x=0.0, y=0.0, yaw=0.0)
            until(lambda: card.runtime.pose_ok, 40, card.info)
            for x in (0.6, 1.0):
                task = call("navigate_to_pose", x=x, y=0.0, yaw=0.0)
                until(lambda: task["nav_id"] in card.receipts, 60, card.info)
                assert card.receipts[task["nav_id"]]["status"] == "arrived", card.receipts
                assert abs(room.x-x) <= 0.25, room.x
            assert room.max_nonzero > 0, "no actual Nav2 nonzero command received"
            call("navigate_to_pose", x=2.0, y=0.0, yaw=0.0)
            room.scan_on = False
            until(lambda: card.runtime.diagnostic == "scan_stale", 3, card.info)
            until(lambda: room.v == room.w == 0, 1, card.info)
            room.scan_on = True
            until(lambda: card.runtime.pose_ok, 5, card.info)
            call("stop_navigation")
            call("stop")
            print("PLANAR_ROS_SMOKE=PASS (idealized room, not hardware acceptance)")
        finally:
            card.stop()
            if card.runtime:
                card.runtime.close()
            executor.shutdown(timeout_sec=3)
            room.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
