#!/usr/bin/env python3
"""Verify Agent Core persisted and discovered the navigation Perception card."""

from __future__ import annotations

import json
import pathlib
import ssl
import time
import urllib.request


def access_token() -> str:
    path = pathlib.Path("/opt/phanthy-motus/.env")
    if not path.exists():
        return ""
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if line.startswith("ACCESS_TOKEN=") and not line.startswith("#"):
            return line.split("=", 1)[1].strip()
    return ""


def fetch_registry() -> list[dict]:
    token = access_token()
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    request = urllib.request.Request(
        "https://127.0.0.1:15678/api/mcp", headers=headers
    )
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        payload = json.load(response)
    if payload.get("code") != 200:
        raise AssertionError(f"unexpected registry response: {payload!r}")
    return payload.get("data", [])


def main() -> None:
    deadline = time.monotonic() + 20.0
    match = None
    while time.monotonic() < deadline:
        items = fetch_registry()
        matches = [
            item
            for item in items
            if item.get("name") == "Perception Stack"
            and item.get("url") == "http://localhost:15720/mcp"
        ]
        if len(matches) == 1:
            tools = matches[0].get("tools") or []
            if any(tool.get("name") == "navigation2" for tool in tools):
                match = matches[0]
                break
        time.sleep(0.5)

    if match is None:
        raise AssertionError("navigation2 was not discovered in Agent Core")

    tools = [tool for tool in match["tools"] if tool.get("name") == "navigation2"]
    assert len(tools) == 1, tools
    tool = tools[0]
    assert tool.get("type") == "processor", tool
    actions = (
        tool.get("inputSchema", {})
        .get("properties", {})
        .get("action", {})
        .get("enum", [])
    )
    assert len(actions) == 14, actions
    output_topics = [entry.get("topic") for entry in tool.get("topic_out", [])]
    assert "/ubuntu/navigation/nav2/cmd_vel_shadow" not in output_topics
    assert output_topics == ["/ubuntu/navigation/nav2/velocity_proposal"], output_topics
    topic_actions = tool.get("x-topic-actions") or []
    assert len(topic_actions) == 1, topic_actions
    assert topic_actions[0].get("schema") == "phanthy.navigation.goal.v1", topic_actions
    proposals = [
        entry
        for entry in tool.get("topic_out", [])
        if entry.get("port") == "velocity_proposal"
    ]
    assert len(proposals) == 1, proposals
    assert proposals[0].get("ros_type") == "std_msgs/msg/String", proposals[0]
    assert (
        proposals[0].get("schema")
        == "phanthy.navigation.velocity_proposal.v1"
    ), proposals[0]
    print(
        json.dumps(
            {
                "mcp_id": match.get("id"),
                "name": match.get("name"),
                "url": match.get("url"),
                "tool": tool.get("name"),
                "type": tool.get("type"),
                "actions": len(actions),
            },
            separators=(",", ":"),
        )
    )
    print("GENERAL_NAVIGATION_CORE_REGISTRY=PASS")


if __name__ == "__main__":
    main()
