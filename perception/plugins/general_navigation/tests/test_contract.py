import unittest

from plugins.general_navigation.contract import (
    GENERAL_NAVIGATION_ACTION_PARAMS,
    GENERAL_NAVIGATION_ACTIONS,
    general_navigation_tool_definition,
)


EXPECTED_ACTION_PARAMS = {
    "start_mapping": ["map_name"],
    "stop_mapping": [],
    "tag_place": ["name", "description"],
    "untag_place": ["name"],
    "list_tags": [],
    "list_maps": [],
    "delete_map": ["map_name"],
    "load_map": ["map_name"],
    "navigate_to_tag": ["tag_name", "speed", "mode"],
    "navigate_to_pose": ["x", "y", "yaw", "speed", "mode"],
    "wait_navigation_done": ["stall_timeout"],
    "pause_nav": [],
    "resume_nav": [],
    "stop_nav": [],
}


class GeneralNavigationContractTest(unittest.TestCase):
    def test_card_uses_general_navigation_name_and_processor_type(self):
        tool = general_navigation_tool_definition("ubuntu")

        self.assertEqual(tool["name"], "navigation")
        self.assertEqual(tool["type"], "processor")
        self.assertFalse(tool["multiInstance"])
        self.assertEqual(f"general_{tool['name']}", "general_navigation")

    def test_contract_preserves_exactly_the_14_frozen_actions(self):
        tool = general_navigation_tool_definition("ubuntu")
        schema = tool["inputSchema"]

        self.assertEqual(len(GENERAL_NAVIGATION_ACTIONS), 14)
        self.assertEqual(
            tuple(schema["properties"]["action"]["enum"]),
            GENERAL_NAVIGATION_ACTIONS,
        )
        self.assertNotIn("info", GENERAL_NAVIGATION_ACTIONS)
        self.assertNotIn("config", GENERAL_NAVIGATION_ACTIONS)
        self.assertNotIn("start", GENERAL_NAVIGATION_ACTIONS)
        self.assertNotIn("stop", GENERAL_NAVIGATION_ACTIONS)
        self.assertEqual(
            {
                action: definition["params"]
                for action, definition in schema["x-action-params"].items()
            },
            EXPECTED_ACTION_PARAMS,
        )
        self.assertEqual(
            schema["x-action-params"], GENERAL_NAVIGATION_ACTION_PARAMS
        )

    def test_topics_are_driver_inputs_and_loco_proposal_output(self):
        tool = general_navigation_tool_definition("ubuntu")

        self.assertEqual(
            [entry["topic"] for entry in tool["topic_in"]],
            [
                "/ubuntu/loco/state",
                "/ubuntu/lidar/cloud",
                "/ubuntu/navigation/goal_pose",
            ],
        )
        self.assertEqual(
            [entry["format"] for entry in tool["topic_in"]],
            ["data/json", "sensor/pointcloud", "data/json"],
        )
        output_topics = [entry["topic"] for entry in tool["topic_out"]]
        self.assertEqual(
            output_topics, ["/ubuntu/navigation/nav2/velocity_proposal"]
        )
        self.assertNotIn(
            "/ubuntu/navigation/nav2/cmd_vel_shadow", output_topics
        )
        self.assertIn(
            "/ubuntu/navigation/nav2/velocity_proposal", output_topics
        )
        self.assertNotIn("/cmd_vel", output_topics)
        mode_schema = tool["inputSchema"]["properties"]["mode"]
        self.assertEqual(mode_schema["enum"], [0])
        self.assertEqual(mode_schema["default"], 0)
        speed_schema = tool["inputSchema"]["properties"]["speed"]
        self.assertEqual(
            (speed_schema["minimum"], speed_schema["maximum"], speed_schema["default"]),
            (0.2, 0.8, 0.5),
        )
        stall_schema = tool["inputSchema"]["properties"]["stall_timeout"]
        self.assertEqual(
            (stall_schema["minimum"], stall_schema["maximum"], stall_schema["default"]),
            (1.0, 3600.0, 90.0),
        )
        loco, lidar, goal = tool["topic_in"]
        self.assertEqual(loco["schema"], "unitree.g1.loco_state.legacy")
        self.assertEqual(
            loco["compatible_schemas"], ["phanthy.g1.loco_state.v2"]
        )
        self.assertIn("adapter contract", loco["frame_id"])
        self.assertEqual(lidar["schema"], "unitree.g1.pointcloud.legacy")
        self.assertEqual(
            lidar["compatible_schemas"], ["phanthy.sensor.pointcloud.v2"]
        )
        self.assertIn("adapter receive time", lidar["timestamp"])
        self.assertEqual(lidar["qos"], "RELIABLE + KEEP_LAST(depth=10) + VOLATILE")
        self.assertFalse(goal["required"])
        self.assertEqual(goal["schema"], "phanthy.navigation.goal.v1")
        self.assertEqual(goal["frame_id"], "map")
        proposal = next(
            entry
            for entry in tool["topic_out"]
            if entry.get("port") == "velocity_proposal"
        )
        self.assertEqual(proposal["format"], "data/json")
        self.assertEqual(proposal["ros_type"], "std_msgs/msg/String")
        self.assertEqual(
            proposal["qos"], "RELIABLE + KEEP_LAST(depth=10) + VOLATILE"
        )
        self.assertEqual(
            proposal["schema"], "phanthy.navigation.velocity_proposal.v1"
        )
        self.assertEqual(proposal["frame_id"], "base_link")
        self.assertEqual(proposal["max_age_ms"], 250)
        control = tool["x-execution-control"]
        self.assertEqual(control["target_tool"], "loco")
        self.assertEqual(control["lease_argument"], "_control_nav_id")
        self.assertEqual(control["output_port"], "velocity_proposal")
        topic_action = tool["x-topic-actions"]
        self.assertEqual(
            topic_action,
            [
                {
                    "port": "goal_pose",
                    "action": "navigate_to_pose",
                    "wait_action": "wait_navigation_done",
                    "stop_action": "stop_nav",
                    "schema": "phanthy.navigation.goal.v1",
                    "id_field": "goal_id",
                    "allowed_fields": ["x", "y", "yaw", "speed", "mode"],
                }
            ],
        )

    def test_each_call_returns_an_independent_contract(self):
        first = general_navigation_tool_definition("ubuntu")
        second = general_navigation_tool_definition("ubuntu")
        first["inputSchema"]["properties"]["action"]["enum"].append("unsafe")

        self.assertNotEqual(first, second)
        self.assertNotIn(
            "unsafe", second["inputSchema"]["properties"]["action"]["enum"]
        )


if __name__ == "__main__":
    unittest.main()
