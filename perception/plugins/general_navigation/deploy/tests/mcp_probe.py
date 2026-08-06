#!/usr/bin/env python3
"""Probe navigation2 on a shared Perception MCP endpoint."""

from __future__ import annotations

import json
import os
import time
import urllib.request


EXPECTED_ACTIONS = [
    "start_mapping",
    "stop_mapping",
    "tag_place",
    "untag_place",
    "list_tags",
    "list_maps",
    "delete_map",
    "load_map",
    "navigate_to_tag",
    "navigate_to_pose",
    "wait_navigation_done",
    "pause_nav",
    "resume_nav",
    "stop_nav",
]


def rpc(method: str, params: dict | None, request_id: int) -> dict:
    url = os.environ.get("MCP_URL", "http://127.0.0.1:15720/mcp")
    body = json.dumps(
        {
            "jsonrpc": "2.0",
            "id": request_id,
            "method": method,
            "params": params or {},
        }
    ).encode()
    request = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        payload = json.load(response)
    if "error" in payload:
        raise AssertionError(f"{method} failed: {payload['error']}")
    return payload["result"]


def initialize() -> dict:
    startup_timeout = float(os.environ.get("MCP_STARTUP_TIMEOUT", "0"))
    if not 0.0 <= startup_timeout <= 60.0:
        raise ValueError("MCP_STARTUP_TIMEOUT must be within [0, 60] seconds")
    deadline = time.monotonic() + startup_timeout
    while True:
        try:
            return rpc(
                "initialize",
                {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {
                        "name": "navigation2-probe",
                        "version": "1",
                    },
                },
                1,
            )
        except OSError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.25)


def read_info(expect_bridge: bool) -> dict:
    deadline = time.monotonic() + (12.0 if expect_bridge else 0.0)
    while True:
        result = rpc(
            "tools/call",
            {
                "name": "navigation2",
                "arguments": {"action": "info"},
            },
            3,
        )
        content = result.get("content", [])
        if len(content) != 1 or content[0].get("type") != "text":
            raise AssertionError(f"unexpected tools/call content: {content!r}")
        info = json.loads(content[0]["text"])
        if not expect_bridge or int(info.get("bridge_subscribers", 0)) >= 1:
            return info
        if time.monotonic() >= deadline:
            raise AssertionError(f"Nav2 bridge subscriber is absent: {info!r}")
        time.sleep(0.5)


def main() -> None:
    initialized = initialize()
    assert initialized["serverInfo"]["name"] == "perception-bundle"

    tools = rpc("tools/list", {}, 2)["tools"]
    matches = [tool for tool in tools if tool.get("name") == "navigation2"]
    assert len(matches) == 1, matches
    tool = matches[0]
    assert tool["type"] == "processor", tool
    assert tool["multiInstance"] is False, tool
    assert tool["inputSchema"]["properties"]["action"]["enum"] == EXPECTED_ACTIONS
    assert tool["inputSchema"]["required"] == ["action"]
    mode_schema = tool["inputSchema"]["properties"]["mode"]
    assert mode_schema.get("enum") == [0], mode_schema
    assert mode_schema.get("default") == 0, mode_schema
    topic_in = tool.get("topic_in", [])
    assert [entry.get("schema") for entry in topic_in] == [
        "unitree.g1.loco_state.legacy",
        "unitree.g1.pointcloud.legacy",
        "phanthy.navigation.goal.v1",
    ], topic_in
    assert topic_in[2].get("required") is False, topic_in[2]
    topic_actions = tool.get("x-topic-actions") or []
    assert len(topic_actions) == 1, topic_actions
    assert topic_actions[0].get("port") == "goal_pose", topic_actions
    assert topic_actions[0].get("action") == "navigate_to_pose", topic_actions
    assert topic_actions[0].get("wait_action") == "wait_navigation_done", topic_actions
    assert topic_actions[0].get("stop_action") == "stop_nav", topic_actions
    execution = tool.get("x-execution-control", {})
    assert execution.get("target_tool") == "loco", execution
    assert execution.get("output_port") == "velocity_proposal", execution
    assert execution.get("lease_argument") == "_control_nav_id", execution
    output_topics = [entry.get("topic") for entry in tool.get("topic_out", [])]
    assert "/ubuntu/navigation/nav2/cmd_vel_shadow" not in output_topics
    assert output_topics == ["/ubuntu/navigation/nav2/velocity_proposal"], output_topics
    proposals = [
        entry
        for entry in tool.get("topic_out", [])
        if entry.get("port") == "velocity_proposal"
    ]
    assert len(proposals) == 1, proposals
    assert proposals[0].get("format") == "data/json", proposals[0]
    assert proposals[0].get("ros_type") == "std_msgs/msg/String", proposals[0]
    assert (
        proposals[0].get("schema")
        == "phanthy.navigation.velocity_proposal.v1"
    ), proposals[0]

    expect_bridge = os.environ.get("EXPECT_BRIDGE_SUBSCRIBER", "0") == "1"
    info = read_info(expect_bridge)
    assert info["backend"] == "nav2_ros_topic", info
    assert info["shadow_only"] is True, info
    assert info["physical_execution"] is False, info
    assert info["actions"] == EXPECTED_ACTIONS, info
    assert info["command_topic"] == "/ubuntu/navigation/nav2/command", info
    assert info["status_topic"] == "/ubuntu/navigation/nav2/status", info
    if os.environ.get("EXPECT_CANVAS_WIRED", "0") == "1":
        assert info.get("canvas_wired") is True, info
        assert all(
            entry.get("connected") is True
            for entry in info.get("topic_in", [])
            if entry.get("required", True)
        ), info

    print(
        json.dumps(
            {
                "tool": tool["name"],
                "type": tool["type"],
                "actions": len(EXPECTED_ACTIONS),
                "backend": info["backend"],
                "bridge_subscribers": info.get("bridge_subscribers", 0),
                "shadow_only": info["shadow_only"],
                "physical_execution": info["physical_execution"],
            },
            separators=(",", ":"),
        )
    )
    print("GENERAL_NAVIGATION_MCP_PROBE=PASS")


if __name__ == "__main__":
    main()
