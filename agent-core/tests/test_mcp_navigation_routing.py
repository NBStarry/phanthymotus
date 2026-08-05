import json
import os
import pathlib
import sys
import tempfile
import unittest
from unittest import mock


_DB_DIR = tempfile.TemporaryDirectory()
os.environ["DB_PATH"] = str(pathlib.Path(_DB_DIR.name) / "data.db")
SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

import mcp_client
import navigation_execution as execution
from api import mcp_manage


PROPOSAL_SCHEMA = "phanthy.navigation.velocity_proposal.v1"
PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
FULL_NAME = "mcp__perception__general_navigation__navigate_to_pose"
CONTROL = {
    "proposal_schema": PROPOSAL_SCHEMA,
    "output_port": "velocity_proposal",
    "target_tool": "loco",
    "lease_argument": "_control_nav_id",
    "start_actions": ["navigate_to_pose"],
    "wait_actions": ["wait_navigation_done"],
    "stop_actions": ["stop_nav"],
    "pause_actions": ["pause_nav"],
    "resume_actions": ["resume_nav"],
    "terminal_statuses": ["arrived", "stopped", "error", "timeout"],
}


def fixture():
    port = {
        "port": "velocity_proposal",
        "topic": PROPOSAL_TOPIC,
        "format": "data/json",
        "schema": PROPOSAL_SCHEMA,
    }
    layout = {
        "cards": [
            {
                "id": "nav-card",
                "mcpId": "perception",
                "toolName": "general_navigation",
                "topicOut": [port],
            },
            {
                "id": "loco-card",
                "mcpId": "driver",
                "toolName": "loco",
                "topicIn": [port],
            },
        ],
        "connections": [
            {
                "fromCardId": "nav-card",
                "fromPortIdx": 0,
                "fromTopic": PROPOSAL_TOPIC,
                "toCardId": "loco-card",
                "toPortIdx": 0,
            }
        ],
    }
    mcps = [
        {
            "id": "perception",
            "transport": "http",
            "url": "http://perception.invalid/mcp",
            "tools": [
                {
                    "name": "general_navigation",
                    "type": "processor",
                    "x-execution-control": CONTROL,
                    "topic_out": [port],
                }
            ],
        },
        {
            "id": "driver",
            "transport": "http",
            "url": "http://driver.invalid/mcp",
            "tools": [
                {
                    "name": "loco",
                    "type": "actuator",
                    "topic_in": [port],
                }
            ],
        },
    ]
    return layout, mcps


class MCPNavigationRoutingTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        execution.reset_state_for_tests()
        mcp_client.registry.clear()
        self.layout, self.mcps = fixture()
        mcp_client.registry.update(
            {
                "perception": {
                    "transport": "http",
                    "url": "http://perception.invalid/mcp",
                    "split_map": {
                        FULL_NAME: {
                            "tool": "general_navigation",
                            "action": "navigate_to_pose",
                        }
                    },
                    "input_schemas": {
                        FULL_NAME: {
                            "type": "object",
                            "properties": {
                                "x": {"type": "number"},
                                "y": {"type": "number"},
                                "yaw": {"type": "number"},
                            },
                            "required": ["x", "y", "yaw"],
                        }
                    },
                    "tool_meta": {FULL_NAME: {"execution_control": CONTROL}},
                },
                "driver": {
                    "transport": "http",
                    "url": "http://driver.invalid/mcp",
                    "split_map": {},
                    "input_schemas": {
                        "mcp__driver__loco": {
                            "type": "object",
                            "properties": {
                                "action": {
                                    "type": "string",
                                    "enum": ["move", "stop"],
                                }
                            },
                            "required": ["action"],
                        }
                    },
                    "tool_meta": {"mcp__driver__loco": {}},
                },
            }
        )

    async def asyncTearDown(self):
        execution.reset_state_for_tests()
        mcp_client.registry.clear()

    async def test_llm_call_binds_driver_before_navigation(self):
        calls = []

        async def fake_raw(name, args, *, validate_arguments=True):
            calls.append((name, dict(args), validate_arguments))
            if name == "mcp__driver__loco":
                return json.dumps(
                    {
                        "state": "ready",
                        "connected": True,
                        "armed": True,
                        "expected_nav_id": args["expected_nav_id"],
                    }
                )
            return json.dumps(
                {
                    "action": "navigate_to_pose",
                    "status": "navigating",
                    "nav_id": args["_control_nav_id"],
                }
            )

        with mock.patch.object(
            execution, "_snapshot", return_value=(self.layout, self.mcps)
        ), mock.patch.object(mcp_client, "_call_tool_raw", side_effect=fake_raw):
            result = json.loads(
                await mcp_client.call_tool(
                    FULL_NAME, {"x": 1.0, "y": 0.0, "yaw": 0.0}
                )
            )

        self.assertEqual(calls[0][0], "mcp__driver__loco")
        self.assertFalse(calls[0][2])
        self.assertEqual(calls[1][0], FULL_NAME)
        self.assertTrue(calls[1][2])
        self.assertEqual(
            calls[0][1]["expected_nav_id"], calls[1][1]["_control_nav_id"]
        )
        self.assertEqual(result["nav_id"], calls[0][1]["expected_nav_id"])

    async def test_canvas_call_uses_same_managed_route(self):
        mcp_entries = [
            {
                "id": "perception",
                "transport": "http",
                "url": "http://perception.invalid/mcp",
            }
        ]
        with mock.patch.object(
            mcp_manage, "_get_mcp_list", return_value=mcp_entries
        ), mock.patch.object(
            mcp_client,
            "execution_control_for_call",
            return_value=(FULL_NAME, CONTROL),
        ), mock.patch.object(
            mcp_client,
            "call_tool",
            new=mock.AsyncMock(return_value='{"status":"navigating"}'),
        ) as managed:
            result = await mcp_manage.mcp_call_tool(
                "perception",
                mcp_manage.MCPCallRequest(
                    tool="general_navigation",
                    arguments={
                        "action": "navigate_to_pose",
                        "x": 1.0,
                        "y": 0.0,
                        "yaw": 0.0,
                    },
                ),
            )

        managed.assert_awaited_once()
        self.assertEqual(result["code"], 200)
        self.assertEqual(
            json.loads(result["data"][0]["text"])["status"], "navigating"
        )

    async def test_tools_endpoint_does_not_require_call_request(self):
        tools = [{"name": "general_navigation"}]
        with mock.patch.object(
            mcp_manage,
            "_get_mcp_list",
            return_value=[
                {
                    "id": "perception",
                    "transport": "stdio",
                    "tools": tools,
                }
            ],
        ):
            result = await mcp_manage.mcp_get_tools("perception")

        self.assertEqual(result, {"code": 200, "data": tools})


if __name__ == "__main__":
    unittest.main()
