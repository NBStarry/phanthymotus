#!/usr/bin/env python3
"""Require the released Driver loco actuator to be ready and unarmed."""

from __future__ import annotations

import json
import pathlib
import ssl
import time
import urllib.request
import urllib.parse


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


def request_json(url: str, body: dict) -> dict:
    headers = {"Content-Type": "application/json"}
    request = urllib.request.Request(
        url,
        data=json.dumps(body).encode(),
        headers=headers,
        method="POST",
    )
    context = None
    if url.startswith("https://"):
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        return json.load(response)


def registry() -> list[dict]:
    headers = {}
    token = access_token()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request("https://127.0.0.1:15678/api/mcp", headers=headers)
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        payload = json.load(response)
    if payload.get("code") != 200:
        raise AssertionError(f"unexpected registry response: {payload!r}")
    return payload.get("data", [])


def rpc(url: str, method: str, params: dict, request_id: int) -> dict:
    payload = request_json(
        url,
        {
            "jsonrpc": "2.0",
            "id": request_id,
            "method": method,
            "params": params,
        },
    )
    if "error" in payload:
        raise AssertionError(f"{method} failed: {payload['error']}")
    return payload["result"]


def decode_tool_result(result: dict) -> dict:
    content = result.get("content", [])
    if len(content) != 1 or content[0].get("type") != "text":
        raise AssertionError(f"unexpected loco info result: {result!r}")
    value = json.loads(content[0]["text"])
    if not isinstance(value, dict):
        raise AssertionError(f"loco info is not an object: {value!r}")
    return value


def main() -> None:
    deadline = time.monotonic() + 20.0
    match = None
    while time.monotonic() < deadline:
        matches = []
        for item in registry():
            tools = item.get("tools") or []
            if any(
                tool.get("name") == "loco" and tool.get("type") == "actuator"
                for tool in tools
            ):
                matches.append(item)
        if len(matches) == 1 and isinstance(matches[0].get("url"), str):
            match = matches[0]
            break
        time.sleep(0.5)
    if match is None:
        raise AssertionError("expected one live Driver loco MCP endpoint")

    url = match["url"]
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme not in {"http", "https"} or parsed.hostname not in {
        "localhost",
        "127.0.0.1",
    }:
        raise AssertionError(f"refusing non-local Driver MCP URL: {url!r}")
    rpc(
        url,
        "initialize",
        {
            "protocolVersion": "2024-11-05",
            "capabilities": {},
            "clientInfo": {"name": "navigation-driver-gate", "version": "1"},
        },
        1,
    )
    tools = rpc(url, "tools/list", {}, 2).get("tools", [])
    candidates = [
        tool["name"]
        for tool in tools
        if tool.get("name") == "loco" or str(tool.get("name", "")).endswith("_loco")
    ]
    if len(candidates) != 1:
        raise AssertionError(f"expected one loco MCP tool, found {candidates!r}")
    info = decode_tool_result(
        rpc(
            url,
            "tools/call",
            {"name": candidates[0], "arguments": {"action": "info"}},
            3,
        )
    )
    assert info.get("state") == "ready", info
    assert info.get("enabled") is True, info
    assert info.get("connected") is False, info
    assert info.get("armed") is False, info
    assert info.get("expected_nav_id") is None, info
    assert info.get("active_nav_id") is None, info
    assert info.get("expected_topic") == PROPOSAL_TOPIC, info
    assert info.get("schema") == PROPOSAL_SCHEMA, info
    ports = info.get("topic_in") or []
    assert len(ports) == 1, ports
    assert ports[0].get("port") == "velocity_proposal", ports
    assert ports[0].get("topic") == PROPOSAL_TOPIC, ports
    assert ports[0].get("schema") == PROPOSAL_SCHEMA, ports

    print(json.dumps(info, separators=(",", ":")))
    print("GENERAL_NAVIGATION_DRIVER_RUNTIME_STANDBY=PASS")


if __name__ == "__main__":
    main()
