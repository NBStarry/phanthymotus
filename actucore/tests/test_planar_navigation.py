"""Run with unittest; no ROS or robot connection is created by these tests."""

import io
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from plugins.planar_navigation.model import ProposalGate, Rejected, Store, config, fresh, goal
from plugins.planar_navigation.parameters import parameters
from plugins.planar_navigation.plugin import PlanarNavigationPlugin, ACTIONS
from plugins.planar_navigation.semantic import decode_rgb, choose


class PlanarTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.cfg = config({"data_dir": self.tmp.name})

    def test_config_boundaries(self):
        for data in ({"max_speed": float("nan")}, {"max_speed": True},
                     {"footprint": [[0, 0], [1, 1]]}, {"execution_enabled": True},
                     {"base_frame": "odom"}, {"foo": 1},
                     {"vlm_url": "http://example.invalid"},
                     {"footprint": [[-1,-1],[1,1],[-1,1],[1,-1]]}):
            with self.subTest(data=data), self.assertRaises(Rejected):
                config(data)
        c = config({"footprint": "[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]"})
        self.assertEqual(len(c["footprint"]), 4)
        with self.assertRaises(Rejected):
            goal({"x": 0, "y": 0, "yaw": 5})

    def test_source_time_not_receive_time(self):
        self.assertFalse(fresh(1, 10, 20, 10.1, 0.5))
        self.assertFalse(fresh(20, 9, 20, 10, 0.5))
        self.assertTrue(fresh(19.9, 9.9, 20, 10, 0.5))
        self.assertFalse(fresh(21, 9.9, 20, 10, 0.5))

    def test_standard_image_and_mirror_commands(self):
        root = Path(__file__).resolve().parents[1]
        text = (root / "Dockerfile.jetson").read_text()
        self.assertIn("FROM bj-warehouse.tencentcloudcr.com/phanthy-motus/jetson-base:jp${JP_VERSION}-torch", text)
        self.assertNotIn("ros-humble-", text)
        self.assertNotIn("trusted=yes", text)
        self.assertIn("Dir::Etc::sourceparts=-", text)
        self.assertIn("COPY actucore/config.yaml /work/config.yaml", text)
        command = "for suite in " + text.split("for suite in ", 1)[1].split(" > /tmp/actucore-ubuntu.list", 1)[0]
        command = command.replace("\\\n", "")
        for codename in ("focal", "jammy"):
            result = subprocess.run(["bash", "-c", command], capture_output=True, text=True,
                                    env={**os.environ, "VERSION_CODENAME": codename,
                                         "APT_MIRROR": "mirrors.tuna.tsinghua.edu.cn"}, check=True)
            lines = result.stdout.splitlines()
            self.assertEqual(len(lines), 3)
            self.assertTrue(all(line.startswith("deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports ") for line in lines))
            self.assertIn(codename + "-security", lines[-1])
        builder = root / "plugins/planar_navigation/build-dependencies.sh"
        rejected = subprocess.run(["bash", str(builder)], env={**os.environ, "BUILD_JOBS": "0"},
                                  capture_output=True, text=True)
        self.assertEqual(rejected.returncode, 2)
        self.assertIn("invalid BUILD_JOBS", rejected.stderr)
        for row in (builder.parent / "sources.lock").read_text().splitlines():
            if row and not row.startswith("#"):
                self.assertRegex(row.split()[2], r"^[0-9a-f]{40}$")

    def test_proposal_stop_and_recovery(self):
        out, clock = [], [10.0]
        gate = ProposalGate(out.append, self.cfg, lambda: clock[0])
        gate.begin("one")
        gate.health("")
        clock[0] += 1.1
        gate.health("")
        gate.command(0.5, 2)
        gate.tick()
        self.assertEqual(out[-1]["velocity"], {"x": 0.2, "y": 0, "yaw": 0.3})
        gate.health("scan_stale")
        self.assertEqual(out[-1]["velocity"]["x"], 0)
        gate.health("")
        clock[0] += 1.1
        gate.health("")
        gate.tick()  # old nonzero must not survive recovery
        self.assertEqual(out[-1]["velocity"]["x"], 0)
        gate.command(-0.1, 0)
        self.assertEqual(out[-1]["velocity"]["x"], 0)
        gate.finish("arrived")
        length = len(out)
        gate.tick()
        self.assertEqual(len(out), length)
        gate.begin("two")
        self.assertEqual(gate.nav_id, "two")

    def test_zero_cannot_be_overtaken_by_copied_candidate(self):
        out = []
        gate = ProposalGate(out.append, self.cfg, lambda: 10)
        gate.begin("race")
        gate.blocker = ""
        gate.command(0.2, 0)
        with gate.lock:
            thread = threading.Thread(target=gate.tick)
            thread.start()
            gate.command(0, 0)
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertTrue(all(x["velocity"]["x"] == 0 for x in out))
        self.assertEqual([x["sequence"] for x in out], list(range(1, len(out)+1)))

    def saved_map(self, store):
        from PIL import Image
        map_id, prefix = store.allocate()
        Image.new("L", (10, 10), 254).save(str(prefix)+".pgm")
        Path(str(prefix)+".yaml").write_text("image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n")
        return store.finalize(map_id, "room")

    def test_map_and_semantic_persist_across_restart(self):
        store = Store(self.tmp.name)
        item = self.saved_map(store)
        point = store.capture(item, "门口", {"x": 1, "y": 2, "yaw": 0}, 123, b"jpeg")
        reopened = Store(self.tmp.name)
        self.assertEqual(reopened.map(item["map_id"]), item)
        self.assertEqual(reopened.points(item["map_id"], item["revision"])[0]["id"], point)
        other = self.saved_map(store)
        self.assertEqual(reopened.points(other["map_id"], other["revision"]), [])
        Path(item["yaml"]).write_text("image: map.pgm\nresolution: 0.1\n")
        with self.assertRaisesRegex(Rejected, "bytes changed"):
            reopened.map(item["map_id"])

    def test_invalid_artifact_never_registered(self):
        store = Store(self.tmp.name)
        ident, prefix = store.allocate()
        Path(str(prefix)+".yaml").write_text("image: /etc/passwd\nresolution: 0.05\n")
        with self.assertRaises(Rejected):
            store.finalize(ident, "bad")
        self.assertEqual(store.maps(), [])

    def test_schema_and_wiring(self):
        card = PlanarNavigationPlugin(self.cfg, None)
        tool = card.get_tools()[0]
        self.assertEqual(set(ACTIONS), set(tool["inputSchema"]["properties"]["action"]["enum"]))
        self.assertTrue(tool["configSchema"]["properties"]["vlm_api_key"]["x-sensitive"])
        self.assertFalse(any(p["port"] == "velocity_proposal" for p in tool["topic_out"]))
        card._wiring({"input_topics": ["/scan", "/odom"]})
        with self.assertRaises(Rejected):
            card._wiring({"input_topics": ["/scan"]})
        with self.assertRaises(Rejected):
            card._wiring({"input_bindings": [{"port": "scan", "topic": "/odom"}]})
        for bindings in ([{}], [{"port": [], "topic": "/scan"}],
                         [{"port": "unknown", "topic": "/scan"}]):
            with self.subTest(bindings=bindings), self.assertRaises(Rejected):
                card._wiring({"input_bindings": bindings})
        with self.assertRaises(Rejected):
            card._wiring({"input_topic": ["/scan"]})
        rgb_card = PlanarNavigationPlugin({**self.cfg, "rgb_topic": "/rgb"}, None)
        with self.assertRaises(Rejected):
            rgb_card._wiring({"input_topics": ["/scan", "/odom"]})
        rgb_card._wiring({"input_topics": ["/scan", "/odom", "/rgb"]})
        result = card.dispatch(card.PREFIX, {"action": "start", "input_topics": ["/scan"]})
        self.assertEqual(result["error_code"], "invalid_canvas_wiring")
        self.assertEqual(card.stop()["state"], "idle")
        self.assertEqual(card.stop()["state"], "idle")
        self.assertFalse(Path(self.tmp.name, "catalog.sqlite").exists())

    def test_mcp_config_rejects_unknown(self):
        card = PlanarNavigationPlugin(self.cfg, None)
        result = card.dispatch(card.PREFIX, {"action": "config", "max_speed": 4})
        self.assertEqual(result["error_code"], "invalid_config")
        self.assertEqual(card.cfg["max_speed"], 0.2)
        result = card.dispatch(card.PREFIX, {"action": "navigate_to_pose", "x": 0, "y": 0, "yaw": 0})
        self.assertEqual(result["error_code"], "not_started")

    def test_parameters_use_standard_controller_without_reverse(self):
        params = parameters(self.cfg, "/data/map.yaml")
        controller = params["controller_server"]["ros__parameters"]
        self.assertEqual(controller["controller_frequency"], 20)
        self.assertFalse(controller["FollowPath"]["allow_reversing"])
        self.assertTrue(controller["FollowPath"]["use_collision_detection"])
        self.assertEqual(params["amcl"]["ros__parameters"]["scan_topic"], "/scan")
        global_map = params["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertFalse(global_map["rolling_window"])
        self.assertTrue(global_map["obstacle_layer"]["scan"]["clearing"])

    def rgb_fixture(self):
        from PIL import Image
        buf = io.BytesIO()
        Image.new("RGB", (2,2), "red").save(buf, "JPEG")
        jpeg = buf.getvalue()
        meta = {"schema": "phanthy.sensor.camera_rgb_frame.v1",
                "header": {"stamp_ns": 123000000000, "frame_id": "camera"},
                "timing": {"source_stamp_ns": 123000000000, "clock_domain": "ros_system_time"},
                "image": {"encoding": "jpeg", "width": 2, "height": 2, "payload_size": len(jpeg)}}
        encoded = json.dumps(meta).encode()
        return struct.pack("<4sII", b"PSE1", len(encoded), len(jpeg))+encoded+jpeg

    def test_pse1_source_contract(self):
        raw = self.rgb_fixture()
        stamp, frame, jpeg = decode_rgb(raw)
        self.assertEqual((stamp, frame), (123, "camera"))
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))
        for bad in (raw[:-1], b"XXXX"+raw[4:], raw.replace(b"ros_system_time", b"device_raw_time")):
            with self.assertRaises(Rejected):
                decode_rgb(bad)

    def test_ambiguous_semantic_result_never_becomes_goal(self):
        points = [{"id": "one", "description": "door", "pose": {"x": 0, "y": 0, "yaw": 0}}]
        self.assertEqual(choose(self.cfg, "door", points)["id"], "one")
        with patch("plugins.planar_navigation.semantic.ask", return_value="invented"):
            with self.assertRaises(Rejected):
                choose(self.cfg, "somewhere", points)


if __name__ == "__main__":
    unittest.main()
