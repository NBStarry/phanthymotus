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

from api import config as config_api
from api import mcp_manage


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


class MemoryConfig:
    def __init__(self, values):
        self.values = values

    def get(self, key, default=None):
        return self.values.get(key, default)

    def __getitem__(self, key):
        return self.values[key]

    def __setitem__(self, key, value):
        self.values[key] = value


def fixture(*, with_execution_wire=True):
    loco_state = {
        "port": "loco_state",
        "topic": "/ubuntu/loco/state",
        "format": "data/json",
        "schema": "unitree.g1.loco_state.legacy",
    }
    lidar = {
        "port": "lidar_cloud",
        "topic": "/ubuntu/lidar/cloud",
        "format": "sensor/pointcloud",
        "schema": "unitree.g1.pointcloud.legacy",
    }
    proposal = {
        "port": "velocity_proposal",
        "topic": PROPOSAL_TOPIC,
        "format": "data/json",
        "schema": PROPOSAL_SCHEMA,
    }
    cards = [
        {
            "id": "state-card",
            "mcpId": "driver",
            "toolName": "loco_state",
            "topicOut": [loco_state],
        },
        {
            "id": "lidar-card",
            "mcpId": "driver",
            "toolName": "lidar_cloud",
            "topicOut": [lidar],
        },
        {
            "id": "nav-card",
            "mcpId": "perception",
            "toolName": "navigation2",
            "topicOut": [proposal],
        },
        {
            "id": "loco-card",
            "mcpId": "driver",
            "toolName": "loco",
            "topicIn": [proposal],
        },
    ]
    connections = [
        {
            "fromCardId": "state-card",
            "fromPortIdx": 0,
            "fromTopic": loco_state["topic"],
            "toCardId": "nav-card",
            "toPortIdx": 0,
        },
        {
            "fromCardId": "lidar-card",
            "fromPortIdx": 0,
            "fromTopic": lidar["topic"],
            "toCardId": "nav-card",
            "toPortIdx": 1,
        },
    ]
    if with_execution_wire:
        connections.append(
            {
                "fromCardId": "nav-card",
                "fromPortIdx": 0,
                "fromTopic": PROPOSAL_TOPIC,
                "toCardId": "loco-card",
                "toPortIdx": 0,
            }
        )
    mcps = [
        {
            "id": "driver",
            "tools": [
                {"name": "loco_state", "type": "sensor", "topic_out": [loco_state]},
                {"name": "lidar_cloud", "type": "sensor", "topic_out": [lidar]},
                {"name": "loco", "type": "actuator", "topic_in": [proposal]},
            ],
        },
        {
            "id": "perception",
            "tools": [
                {
                    "name": "navigation2",
                    "type": "processor",
                    "topic_out": [proposal],
                    "x-execution-control": CONTROL,
                }
            ],
        },
    ]
    return {"cards": cards, "connections": connections}, mcps


def call_result(payload):
    return {
        "code": 200,
        "data": [{"type": "text", "text": json.dumps(payload)}],
    }


class ProjectNavigationLifecycleTest(unittest.IsolatedAsyncioTestCase):
    async def test_invalid_execution_wire_fails_before_any_card_start(self):
        layout, mcps = fixture(with_execution_wire=False)
        memory = MemoryConfig(
            {"canvas_layout": layout, "services": {"mcp": mcps}, "core": {}}
        )
        invoke = mock.AsyncMock()
        events = mock.AsyncMock()

        with mock.patch.object(config_api.config, "main", memory), mock.patch.object(
            mcp_manage, "mcp_call_tool", new=invoke
        ), mock.patch("api.motus_stream.push_event", new=events):
            result = await config_api._do_start_project()

        self.assertFalse(result)
        invoke.assert_not_awaited()
        self.assertFalse(memory.get("core", {}).get("project_running", False))

    async def test_unconfirmed_driver_stop_keeps_project_running(self):
        layout, mcps = fixture()
        memory = MemoryConfig(
            {
                "canvas_layout": layout,
                "services": {"mcp": mcps},
                "core": {"project_running": True},
            }
        )

        async def invoke(mcp_id, req):
            if req.tool == "loco":
                return call_result(
                    {"state": "error", "connected": False, "stop_confirmed": False}
                )
            return call_result({"state": "idle"})

        with mock.patch.object(config_api.config, "main", memory), mock.patch.object(
            mcp_manage, "mcp_call_tool", side_effect=invoke
        ), mock.patch("api.motus_stream.push_event", new=mock.AsyncMock()):
            result = await config_api._do_stop_project()

        self.assertFalse(result)
        self.assertTrue(memory["core"]["project_running"])


if __name__ == "__main__":
    unittest.main()
