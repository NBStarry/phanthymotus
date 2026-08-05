import unittest

from plugins.general_navigation.core import NavigationBackendError
from plugins.general_navigation.plugin import GeneralNavigationPlugin


class FakeBackend:
    def __init__(self):
        self.calls = []
        self.stopped = False

    def info(self):
        return {
            "state": "idle",
            "backend": "fake",
            "shadow_only": True,
            "physical_execution": False,
        }

    def execute(self, action, args, *, nav_id):
        self.calls.append((action, dict(args), nav_id))
        if action in {
            "start_mapping",
            "stop_mapping",
            "tag_place",
            "untag_place",
            "list_tags",
            "list_maps",
            "delete_map",
            "load_map",
            "navigate_to_tag",
        }:
            raise NavigationBackendError("n3_not_ready", "N3 is not ready")
        if action == "navigate_to_pose":
            return {
                "status": "navigating",
                "requested_speed": args["speed"],
                "mode": args["mode"],
            }
        if action == "wait_navigation_done":
            return {"status": "arrived", "stall_timeout": args["stall_timeout"]}
        if action == "pause_nav":
            return {"status": "paused"}
        if action == "resume_nav":
            return {"status": "navigating"}
        if action == "stop_nav":
            return {"status": "stopped"}
        raise AssertionError(action)

    def stop(self):
        self.stopped = True


class FakeNav2Backend(FakeBackend):
    def __init__(self, *, subscribers=1, n3_ready=True):
        super().__init__()
        self.subscribers = subscribers
        self.n3_ready = n3_ready

    def info(self):
        return {
            "state": "idle",
            "backend": "nav2_ros_topic",
            "bridge_subscribers": self.subscribers,
            "n3_ready": self.n3_ready,
            "readiness_blockers": [] if self.n3_ready else ["scan_stale"],
            "shadow_only": True,
            "physical_execution": False,
        }


class GeneralNavigationPluginTest(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.plugin = GeneralNavigationPlugin(
            {}, "ubuntu", None, backend=self.backend
        )
        started = self.plugin.dispatch(
            "navigation",
            {
                "action": "start",
                "instance_id": "card-general-navigation",
                "input_topics": [
                    "/ubuntu/lidar/cloud",
                    "/ubuntu/loco/state",
                ],
            },
        )
        self.assertEqual(started["state"], "ready")

    def test_prefix_and_tool_name_route_to_general_navigation(self):
        tool = self.plugin.get_tools()[0]

        self.assertEqual(self.plugin.PREFIX, "general")
        self.assertEqual(tool["name"], "navigation")
        self.assertEqual(
            f"{self.plugin.PREFIX}_{tool['name']}", "general_navigation"
        )
        self.assertIsNone(self.plugin.dispatch("other", {"action": "list_maps"}))

    def test_hidden_info_does_not_change_the_14_action_schema(self):
        tool = self.plugin.get_tools()[0]
        result = self.plugin.dispatch("navigation", {"action": "info"})

        self.assertEqual(result["backend"], "fake")
        self.assertEqual(result["type"], "processor")
        self.assertTrue(result["canvas_wired"])
        self.assertEqual(len(result["actions"]), 14)
        self.assertEqual(
            result["topic_out"][0]["topic"],
            "/ubuntu/navigation/nav2/velocity_proposal",
        )
        self.assertEqual(
            tool["x-execution-control"]["proposal_schema"],
            "phanthy.navigation.velocity_proposal.v1",
        )
        self.assertNotIn("_control_nav_id", tool["inputSchema"]["properties"])

    def test_trusted_agent_core_nav_id_is_used_without_exposing_public_parameter(self):
        started = self.plugin.dispatch(
            "navigation",
            {
                "action": "navigate_to_pose",
                "x": 1,
                "y": 0,
                "yaw": 0,
                "_control_nav_id": "agent-core-nav-lease",
            },
        )

        self.assertEqual(started["nav_id"], "agent-core-nav-lease")
        self.assertEqual(self.backend.calls[0][2], "agent-core-nav-lease")

    def test_navigation_is_nonblocking_and_wait_uses_same_nav_id(self):
        started = self.plugin.dispatch(
            "navigation",
            {
                "action": "navigate_to_pose",
                "x": 1,
                "y": 2,
                "yaw": 0.3,
                "speed": 0.4,
            },
        )
        waiting = self.plugin.dispatch(
            "navigation", {"action": "wait_navigation_done"}
        )

        self.assertEqual(started["status"], "navigating")
        self.assertTrue(started["nav_id"])
        self.assertEqual(waiting["status"], "arrived")
        self.assertEqual(waiting["nav_id"], started["nav_id"])
        self.assertEqual(waiting["stall_timeout"], 90.0)
        self.assertEqual(started["mode"], 0)
        self.assertEqual(self.backend.calls[0][1]["mode"], 0)
        self.assertEqual(self.backend.calls[0][2], started["nav_id"])
        self.assertEqual(self.backend.calls[1][2], started["nav_id"])

    def test_second_navigation_is_rejected_until_first_is_terminal(self):
        args = {
            "action": "navigate_to_pose",
            "x": 1,
            "y": 2,
            "yaw": 0,
            "mode": 0,
        }
        first = self.plugin.dispatch("navigation", args)
        second = self.plugin.dispatch("navigation", args)

        self.assertEqual(first["status"], "navigating")
        self.assertEqual(second["status"], "error")
        self.assertEqual(second["error_code"], "navigation_active")

    def test_validation_fails_closed_before_backend_call(self):
        cases = [
            (
                {"action": "navigate_to_pose", "y": 0, "yaw": 0},
                "missing_argument",
            ),
            (
                {
                    "action": "navigate_to_pose",
                    "x": float("nan"),
                    "y": 0,
                    "yaw": 0,
                },
                "invalid_argument",
            ),
            (
                {
                    "action": "navigate_to_pose",
                    "x": 0,
                    "y": 0,
                    "yaw": 0,
                    "speed": 0.1,
                },
                "invalid_argument",
            ),
            (
                {
                    "action": "navigate_to_pose",
                    "x": 0,
                    "y": 0,
                    "yaw": 0,
                    "mode": True,
                },
                "invalid_argument",
            ),
            ({"action": "load_map", "map_name": "../unsafe"}, "invalid_argument"),
        ]

        for args, code in cases:
            with self.subTest(args=args):
                result = self.plugin.dispatch("navigation", args)
                self.assertEqual(result["status"], "error")
                self.assertEqual(result["error_code"], code)
        self.assertEqual(self.backend.calls, [])

    def test_unimplemented_n3_action_reports_explicit_gate(self):
        result = self.plugin.dispatch(
            "navigation", {"action": "start_mapping", "map_name": "office"}
        )

        self.assertEqual(result["status"], "error")
        self.assertEqual(result["error_code"], "n3_not_ready")

    def test_stop_is_idempotent_when_idle(self):
        result = self.plugin.dispatch("navigation", {"action": "stop_nav"})

        self.assertEqual(result["status"], "stopped")
        self.assertTrue(result["already_idle"])
        self.assertEqual(self.backend.calls, [])

    def test_disabled_backend_is_visible_instead_of_crashing_plugin_load(self):
        plugin = GeneralNavigationPlugin(
            {"backend": "disabled"}, "ubuntu", None
        )
        started = plugin.dispatch(
            "navigation",
            {
                "action": "start",
                "input_topics": [
                    "/ubuntu/loco/state",
                    "/ubuntu/lidar/cloud",
                ],
            },
        )

        info = plugin.dispatch("navigation", {"action": "info"})
        result = plugin.dispatch("navigation", {"action": "list_maps"})
        self.assertEqual(started["state"], "error")
        self.assertEqual(started["error_code"], "backend_not_ready")
        self.assertEqual(info["state"], "unavailable")
        self.assertFalse(info["canvas_wired"])
        self.assertEqual(result["error_code"], "canvas_not_started")

    def test_canvas_lifecycle_requires_both_frozen_driver_inputs(self):
        plugin = GeneralNavigationPlugin(
            {}, "ubuntu", None, backend=FakeBackend()
        )

        unwired = plugin.dispatch(
            "navigation",
            {
                "action": "navigate_to_pose",
                "x": 1,
                "y": 0,
                "yaw": 0,
            },
        )
        missing = plugin.dispatch(
            "navigation",
            {"action": "start", "input_topic": "/ubuntu/loco/state"},
        )
        ready = plugin.dispatch(
            "navigation",
            {
                "action": "start",
                "instance_id": "card-nav",
                "input_topics": [
                    "/ubuntu/lidar/cloud",
                    "/ubuntu/loco/state",
                ],
            },
        )
        stopped = plugin.dispatch("navigation", {"action": "stop"})

        self.assertEqual(unwired["error_code"], "canvas_not_started")
        self.assertEqual(missing["error_code"], "invalid_canvas_wiring")
        self.assertEqual(ready["state"], "ready")
        self.assertTrue(ready["canvas_wired"])
        self.assertEqual(stopped["state"], "idle")
        self.assertFalse(stopped["canvas_wired"])

    def test_canvas_start_requires_live_nav2_receipt(self):
        inputs = {
            "action": "start",
            "input_topics": ["/ubuntu/lidar/cloud", "/ubuntu/loco/state"],
        }
        no_bridge = GeneralNavigationPlugin(
            {}, "ubuntu", None, backend=FakeNav2Backend(subscribers=0)
        ).dispatch("navigation", inputs)
        stale = GeneralNavigationPlugin(
            {}, "ubuntu", None, backend=FakeNav2Backend(n3_ready=False)
        ).dispatch("navigation", inputs)

        self.assertEqual(no_bridge["error_code"], "nav2_companion_unavailable")
        self.assertEqual(stale["error_code"], "navigation_not_ready")
        self.assertIn("scan_stale", stale["message"])

    def test_optional_goal_binding_is_visible_without_replacing_sensor_inputs(self):
        plugin = GeneralNavigationPlugin(
            {}, "ubuntu", None, backend=FakeBackend()
        )
        result = plugin.dispatch(
            "navigation",
            {
                "action": "start",
                "input_bindings": [
                    {"port": "loco_state", "topic": "/ubuntu/loco/state"},
                    {"port": "lidar", "topic": "/ubuntu/lidar/cloud"},
                    {"port": "goal_pose", "topic": "/planner/goal"},
                ],
            },
        )

        self.assertEqual(result["state"], "ready")
        by_port = {entry["port"]: entry for entry in result["topic_in"]}
        self.assertTrue(by_port["loco_state"]["connected"])
        self.assertTrue(by_port["lidar"]["connected"])
        self.assertTrue(by_port["goal_pose"]["connected"])

    def test_duplicate_input_port_bindings_fail_closed(self):
        plugin = GeneralNavigationPlugin(
            {}, "ubuntu", None, backend=FakeBackend()
        )
        result = plugin.dispatch(
            "navigation",
            {
                "action": "start",
                "input_bindings": [
                    {"port": "loco_state", "topic": "/ubuntu/loco/state"},
                    {"port": "loco_state", "topic": "/ubuntu/loco/state"},
                    {"port": "lidar", "topic": "/ubuntu/lidar/cloud"},
                ],
            },
        )

        self.assertEqual(result["error_code"], "invalid_canvas_wiring")
        self.assertIn("duplicate ports", result["message"])

    def test_stop_releases_backend(self):
        self.plugin.stop()
        self.assertTrue(self.backend.stopped)


if __name__ == "__main__":
    unittest.main()
