import json
import pathlib
import sys
import unittest
from unittest import mock


SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

import topic_action_routing as routing
import config
from api import mcp_manage


GOAL_SCHEMA = "phanthy.navigation.goal.v1"
GOAL_TOPIC = "/planner/goal"


def fixture(*, connected=True, source_schema=GOAL_SCHEMA):
    source_output = {
        "port": "goal",
        "topic": GOAL_TOPIC,
        "format": "data/json",
        "schema": source_schema,
    }
    goal_input = {
        "port": "goal_pose",
        "topic": "/ubuntu/navigation/goal_pose",
        "format": "data/json",
        "schema": GOAL_SCHEMA,
        "required": False,
    }
    declaration = {
        "port": "goal_pose",
        "action": "navigate_to_pose",
        "wait_action": "wait_navigation_done",
        "stop_action": "stop_nav",
        "schema": GOAL_SCHEMA,
        "id_field": "goal_id",
        "allowed_fields": ["x", "y", "yaw", "speed", "mode"],
    }
    layout = {
        "cards": [
            {
                "id": "goal-card",
                "mcpId": "planner",
                "toolName": "goal_source",
                "topicOut": [source_output],
            },
            {
                "id": "nav-card",
                "mcpId": "perception",
                "toolName": "navigation2",
                "topicIn": [goal_input],
            },
        ],
        "connections": [],
    }
    if connected:
        layout["connections"].append(
            {
                "fromCardId": "goal-card",
                "fromPortIdx": 0,
                "fromTopic": GOAL_TOPIC,
                "toCardId": "nav-card",
                "toPortIdx": 0,
                "format": "data/json",
            }
        )
    mcps = [
        {
            "id": "planner",
            "tools": [
                {
                    "name": "goal_source",
                    "type": "processor",
                    "topic_out": [source_output],
                }
            ],
        },
        {
            "id": "perception",
            "tools": [
                {
                    "name": "navigation2",
                    "type": "processor",
                    "topic_in": [goal_input],
                    "x-topic-actions": [declaration],
                }
            ],
        },
    ]
    return layout, mcps


class TopicActionRoutingTest(unittest.IsolatedAsyncioTestCase):
    def tearDown(self):
        routing._active_routes.clear()
        routing._seen_ids.clear()

    def test_optional_unconnected_input_creates_no_route(self):
        layout, mcps = fixture(connected=False)

        routes = routing.resolve_topic_action_routes(
            layout=layout, mcp_entries=mcps
        )

        self.assertEqual(routes, [])

    def test_exact_schema_connection_resolves_route(self):
        layout, mcps = fixture()

        routes = routing.resolve_topic_action_routes(
            layout=layout, mcp_entries=mcps
        )

        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0].topic, GOAL_TOPIC)
        self.assertEqual(routes[0].action, "navigate_to_pose")
        self.assertEqual(routes[0].wait_action, "wait_navigation_done")
        self.assertEqual(routes[0].stop_action, "stop_nav")

    def test_source_schema_mismatch_fails_closed(self):
        layout, mcps = fixture(source_schema="unknown.goal")

        with self.assertRaises(routing.TopicActionError):
            routing.resolve_topic_action_routes(layout=layout, mcp_entries=mcps)

    def test_goal_decoder_accepts_only_public_navigation_fields(self):
        layout, mcps = fixture()
        route = routing.resolve_topic_action_routes(
            layout=layout, mcp_entries=mcps
        )[0]
        goal_id, arguments = routing._decode_goal(
            route,
            json.dumps(
                {
                    "schema": GOAL_SCHEMA,
                    "goal_id": "goal-1",
                    "x": 1.0,
                    "y": 2.0,
                    "yaw": 0.5,
                    "speed": 0.4,
                    "mode": 0,
                }
            ).encode(),
        )

        self.assertEqual(goal_id, "goal-1")
        self.assertEqual(arguments["action"], "navigate_to_pose")
        self.assertNotIn("goal_id", arguments)
        with self.assertRaises(routing.TopicActionError):
            routing._decode_goal(
                route,
                json.dumps(
                    {
                        "schema": GOAL_SCHEMA,
                        "goal_id": "goal-2",
                        "x": 1.0,
                        "y": 2.0,
                        "yaw": 0.0,
                        "_control_nav_id": "bypass",
                    }
                ).encode(),
            )

    async def test_runtime_subscription_is_reliable(self):
        layout, mcps = fixture()
        bridge = mock.Mock()
        bridge.subscribe.return_value = True

        with mock.patch.dict(sys.modules, {"ros2_bridge": bridge}):
            routes = await routing.start_topic_action_routes(
                layout=layout, mcp_entries=mcps
            )

        self.assertEqual(len(routes), 1)
        self.assertTrue(bridge.subscribe.call_args.kwargs["reliable"])

    async def test_one_goal_waits_for_terminal_driver_release(self):
        layout, mcps = fixture()
        route = routing.resolve_topic_action_routes(
            layout=layout, mcp_entries=mcps
        )[0]
        calls = []

        async def invoke(_mcp_id, request):
            calls.append(dict(request.arguments))
            if request.arguments["action"] == "navigate_to_pose":
                payload = {"status": "navigating", "nav_id": "nav-1"}
            else:
                payload = {
                    "status": "arrived",
                    "execution": {"stop_confirmed": True},
                }
            return {
                "code": 200,
                "data": [{"type": "text", "text": json.dumps(payload)}],
            }

        message = json.dumps(
            {
                "schema": GOAL_SCHEMA,
                "goal_id": "goal-terminal",
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
            }
        ).encode()
        with mock.patch.object(
            config, "main", {"core": {"project_running": True}}
        ), mock.patch.object(
            mcp_manage, "mcp_call_tool", side_effect=invoke
        ), mock.patch(
            "api.motus_stream.push_event", new=mock.AsyncMock()
        ):
            await routing._handle_message(route, message)

        self.assertEqual(
            [call["action"] for call in calls],
            ["navigate_to_pose", "wait_navigation_done"],
        )
        self.assertEqual(calls[1]["stall_timeout"], 90.0)

    async def test_missing_ros_subscription_fails_project_start(self):
        layout, mcps = fixture()
        bridge = mock.Mock()
        bridge.subscribe.return_value = False

        with mock.patch.dict(sys.modules, {"ros2_bridge": bridge}):
            with self.assertRaises(routing.TopicActionError):
                await routing.start_topic_action_routes(
                    layout=layout, mcp_entries=mcps
                )


if __name__ == "__main__":
    unittest.main()
