import json
import pathlib
import sys
import unittest
from unittest import mock


SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

import navigation_execution as execution


PROPOSAL_SCHEMA = "phanthy.navigation.velocity_proposal.v1"
PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
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


def fixture(*, with_connection=True):
    proposal_port = {
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
                "toolName": "navigation2",
                "topicOut": [proposal_port],
            },
            {
                "id": "loco-card",
                "mcpId": "driver",
                "toolName": "loco",
                "topicIn": [proposal_port],
            },
        ],
        "connections": [],
    }
    if with_connection:
        layout["connections"].append(
            {
                "fromCardId": "nav-card",
                "fromPortIdx": 0,
                "fromTopic": PROPOSAL_TOPIC,
                "toCardId": "loco-card",
                "toPortIdx": 0,
                "format": "data/json",
            }
        )
    mcps = [
        {
            "id": "perception",
            "tools": [
                {
                    "name": "navigation2",
                    "type": "processor",
                    "x-execution-control": CONTROL,
                    "topic_out": [proposal_port],
                }
            ],
        },
        {
            "id": "driver",
            "tools": [
                {
                    "name": "loco",
                    "type": "actuator",
                    "topic_in": [proposal_port],
                }
            ],
        },
    ]
    return layout, mcps


class NavigationExecutionResolverTest(unittest.TestCase):
    def test_resolves_exact_navigation_to_loco_wire(self):
        layout, mcps = fixture()

        link = execution.resolve_execution_link(
            layout=layout,
            mcp_entries=mcps,
            source_mcp_id="perception",
            source_tool="navigation2",
            control=CONTROL,
        )

        self.assertEqual(link.target_mcp_id, "driver")
        self.assertEqual(link.target_tool, "loco")
        self.assertEqual(link.proposal_topic, PROPOSAL_TOPIC)

    def test_missing_driver_wire_fails_closed(self):
        layout, mcps = fixture(with_connection=False)

        with self.assertRaises(execution.ExecutionControlError) as raised:
            execution.resolve_execution_link(
                layout=layout,
                mcp_entries=mcps,
                source_mcp_id="perception",
                source_tool="navigation2",
                control=CONTROL,
            )

        self.assertEqual(
            raised.exception.code, "execution_driver_connection_invalid"
        )

class NavigationExecutionLifecycleTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        execution.reset_state_for_tests()
        self.layout, self.mcps = fixture()
        self.calls = []
        self.stop_confirmed = True

    async def asyncTearDown(self):
        execution.reset_state_for_tests()

    async def invoke(self, mcp_id, tool, args):
        self.calls.append((mcp_id, tool, dict(args)))
        action = args.get("action")
        if (mcp_id, tool, action) == ("driver", "loco", "start"):
            return json.dumps(
                {
                    "state": "ready",
                    "connected": True,
                    "armed": True,
                    "expected_nav_id": args["expected_nav_id"],
                }
            )
        if (mcp_id, tool, action) == ("driver", "loco", "stop"):
            return json.dumps(
                {
                    "state": "idle" if self.stop_confirmed else "error",
                    "connected": not self.stop_confirmed,
                    "stop_confirmed": self.stop_confirmed,
                }
            )
        if action == "navigate_to_pose":
            return json.dumps(
                {
                    "action": action,
                    "status": "navigating",
                    "nav_id": args["_control_nav_id"],
                }
            )
        if action == "wait_navigation_done":
            return json.dumps({"action": action, "status": "arrived"})
        if action == "pause_nav":
            return json.dumps({"action": action, "status": "paused"})
        raise AssertionError((mcp_id, tool, args))

    async def call(self, action, arguments):
        with mock.patch.object(
            execution, "_snapshot", return_value=(self.layout, self.mcps)
        ):
            return await execution.call_with_execution_lease(
                source_mcp_id="perception",
                source_tool="navigation2",
                action=action,
                arguments={"action": action, **arguments},
                control=CONTROL,
                invoke=self.invoke,
            )

    async def test_binds_driver_before_goal_and_releases_after_arrival(self):
        started = json.loads(
            await self.call(
                "navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0}
            )
        )
        finished = json.loads(await self.call("wait_navigation_done", {}))

        bind = self.calls[0]
        navigation = self.calls[1]
        stop = self.calls[-1]
        self.assertEqual(bind[:2], ("driver", "loco"))
        self.assertEqual(bind[2]["action"], "start")
        self.assertEqual(
            bind[2]["expected_nav_id"], navigation[2]["_control_nav_id"]
        )
        self.assertEqual(started["nav_id"], bind[2]["expected_nav_id"])
        self.assertEqual(started["execution"]["state"], "armed")
        self.assertEqual(stop[:2], ("driver", "loco"))
        self.assertEqual(stop[2]["action"], "stop")
        self.assertTrue(finished["execution"]["stop_confirmed"])

    async def test_missing_wire_never_calls_navigation_or_driver(self):
        self.layout, self.mcps = fixture(with_connection=False)

        result = json.loads(
            await self.call(
                "navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0}
            )
        )

        self.assertEqual(
            result["error_code"], "execution_driver_connection_invalid"
        )
        self.assertEqual(self.calls, [])

    async def test_unconfirmed_driver_stop_overrides_navigation_success(self):
        await self.call(
            "navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0}
        )
        self.stop_confirmed = False

        result = json.loads(await self.call("wait_navigation_done", {}))

        self.assertEqual(
            result["error_code"], "execution_driver_stop_unconfirmed"
        )
        self.assertEqual(result["navigation_result"]["status"], "arrived")

    async def test_pause_retires_lease_and_resume_fails_closed(self):
        await self.call(
            "navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0}
        )
        paused = json.loads(await self.call("pause_nav", {}))
        resumed = json.loads(await self.call("resume_nav", {}))

        self.assertEqual(paused["execution"]["state"], "released")
        self.assertEqual(resumed["error_code"], "execution_lease_retired")


if __name__ == "__main__":
    unittest.main()
