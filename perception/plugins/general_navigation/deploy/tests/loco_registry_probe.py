#!/usr/bin/env python3
"""Verify that Agent Core discovered the Driver loco navigation input."""

from __future__ import annotations

import json
import pathlib
import ssl
import time
import urllib.request


PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
PROPOSAL_SCHEMA = "phanthy.navigation.velocity_proposal.v1"


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
    matches: list[tuple[dict, dict]] = []
    while time.monotonic() < deadline:
        matches = []
        for item in fetch_registry():
            for tool in item.get("tools") or []:
                if tool.get("name") == "loco" and tool.get("type") == "actuator":
                    matches.append((item, tool))
        if len(matches) == 1:
            break
        time.sleep(0.5)

    if len(matches) != 1:
        raise AssertionError(
            f"expected one registered loco actuator, found {len(matches)}"
        )
    owner, tool = matches[0]
    inputs = [
        entry
        for entry in tool.get("topic_in", [])
        if entry.get("port") == "velocity_proposal"
    ]
    assert len(inputs) == 1, inputs
    proposal = inputs[0]
    assert proposal.get("topic") == PROPOSAL_TOPIC, proposal
    assert proposal.get("format") == "data/json", proposal
    assert proposal.get("ros_type") == "std_msgs/msg/String", proposal
    assert proposal.get("schema") == PROPOSAL_SCHEMA, proposal

    print(
        json.dumps(
            {
                "mcp_id": owner.get("id"),
                "name": owner.get("name"),
                "tool": tool.get("name"),
                "type": tool.get("type"),
                "input_port": proposal.get("port"),
                "input_topic": proposal.get("topic"),
                "schema": proposal.get("schema"),
            },
            separators=(",", ":"),
        )
    )
    print("GENERAL_NAVIGATION_LOCO_REGISTRY=PASS")


if __name__ == "__main__":
    main()
