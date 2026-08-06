#!/usr/bin/env python3
"""Plan or apply the four-card navigation canvas without touching unrelated cards."""

from __future__ import annotations

import json
import os
import pathlib
import ssl
import time
import urllib.request
import uuid


NAVIGATION_TOOL = "navigation2"
LEGACY_NAVIGATION_TOOL = "general_navigation"
MANAGED_TOOLS = ("loco_state", "lidar_cloud", NAVIGATION_TOOL, "loco")
STATE_TOPIC = "/ubuntu/loco/state"
LIDAR_TOPIC = "/ubuntu/lidar/cloud"
PROPOSAL_TOPIC = "/ubuntu/navigation/nav2/velocity_proposal"
GOAL_SCHEMA = "phanthy.navigation.goal.v1"


def access_token() -> str:
    path = pathlib.Path("/opt/phanthy-motus/.env")
    if not path.exists():
        return ""
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if line.startswith("ACCESS_TOKEN=") and not line.startswith("#"):
            return line.split("=", 1)[1].strip()
    return ""


CONTEXT = ssl.create_default_context()
CONTEXT.check_hostname = False
CONTEXT.verify_mode = ssl.CERT_NONE


def request(path: str, body: dict | None = None) -> dict:
    headers = {}
    token = access_token()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    data = None
    method = "GET"
    if body is not None:
        data = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
        method = "POST"
    req = urllib.request.Request(
        f"https://127.0.0.1:15678/api/{path}",
        data=data,
        headers=headers,
        method=method,
    )
    with urllib.request.urlopen(req, timeout=5, context=CONTEXT) as response:
        return json.load(response)


def registry_tools() -> dict[str, tuple[dict, dict]]:
    deadline = time.monotonic() + 20.0
    while True:
        found: dict[str, list[tuple[dict, dict]]] = {
            name: [] for name in MANAGED_TOOLS
        }
        for mcp in request("mcp").get("data", []):
            for tool in mcp.get("tools") or []:
                name = tool.get("name") if isinstance(tool, dict) else ""
                if name in found:
                    found[name].append((mcp, tool))
        if all(len(found[name]) == 1 for name in MANAGED_TOOLS):
            return {name: found[name][0] for name in MANAGED_TOOLS}
        if time.monotonic() >= deadline:
            counts = {name: len(entries) for name, entries in found.items()}
            raise AssertionError(f"expected one live MCP tool per card: {counts}")
        time.sleep(0.5)


def port_index(ports: list, *, topic: str = "", port: str = "") -> int:
    indexes = [
        index
        for index, item in enumerate(ports)
        if isinstance(item, dict)
        and (not topic or item.get("topic") == topic)
        and (not port or item.get("port") == port)
    ]
    assert len(indexes) == 1, (topic, port, ports)
    return indexes[0]


def build_layout(layout: dict, tools: dict[str, tuple[dict, dict]]) -> dict:
    cards = [dict(card) for card in layout.get("cards") or []]
    existing_navigation = next(
        (
            card
            for card in cards
            if card.get("toolName")
            in {NAVIGATION_TOOL, LEGACY_NAVIGATION_TOOL}
        ),
        None,
    )
    preserved_goal_connections = []
    if existing_navigation is not None:
        old_goal_indexes = [
            index
            for index, port in enumerate(existing_navigation.get("topicIn") or [])
            if isinstance(port, dict) and port.get("port") == "goal_pose"
        ]
        if old_goal_indexes:
            assert len(old_goal_indexes) == 1, "duplicate goal_pose input ports"
            preserved_goal_connections = [
                dict(connection)
                for connection in layout.get("connections") or []
                if connection.get("toCardId") == existing_navigation.get("id")
                and int(connection.get("toPortIdx", -1)) == old_goal_indexes[0]
            ]
            assert len(preserved_goal_connections) <= 1, (
                "connect at most one source to goal_pose"
            )
    selected = {}
    base_x = max([float(card.get("x", 0)) for card in cards] + [0.0]) + 320.0
    positions = {
        "loco_state": (base_x, 80.0),
        "lidar_cloud": (base_x, 300.0),
        NAVIGATION_TOOL: (base_x + 340.0, 190.0),
        "loco": (base_x + 680.0, 190.0),
    }

    for name in MANAGED_TOOLS:
        aliases = (
            {NAVIGATION_TOOL, LEGACY_NAVIGATION_TOOL}
            if name == NAVIGATION_TOOL
            else {name}
        )
        matches = [card for card in cards if card.get("toolName") in aliases]
        assert len(matches) <= 1, f"duplicate {name} canvas cards"
        mcp, tool = tools[name]
        if matches:
            card = matches[0]
        else:
            x, y = positions[name]
            card = {
                "id": f"card-general-navigation-{name.replace('_', '-')}",
                "x": x,
                "y": y,
            }
            cards.append(card)
        card.update(
            {
                "mcpId": mcp["id"],
                "toolName": name,
                "driverName": mcp.get("server_name") or mcp.get("name") or mcp["id"],
                "topicIn": tool.get("topic_in") or [],
                "topicOut": tool.get("topic_out") or [],
            }
        )
        selected[name] = card

    managed_ids = {card["id"] for card in selected.values()}
    connections = [
        connection
        for connection in layout.get("connections") or []
        if connection.get("fromCardId") not in managed_ids
        and connection.get("toCardId") not in managed_ids
    ]

    def connect(source: str, target: str, topic: str, *, source_port: str = ""):
        source_card = selected[source]
        target_card = selected[target]
        source_index = port_index(
            source_card["topicOut"], topic=topic, port=source_port
        )
        target_index = port_index(target_card["topicIn"], topic=topic)
        output = source_card["topicOut"][source_index]
        connections.append(
            {
                "id": f"conn-general-navigation-{source}-{target}",
                "fromCardId": source_card["id"],
                "fromPortIdx": source_index,
                "toCardId": target_card["id"],
                "toPortIdx": target_index,
                "format": output.get("format", ""),
                "fromTopic": topic,
            }
        )

    connect("loco_state", NAVIGATION_TOOL, STATE_TOPIC)
    connect("lidar_cloud", NAVIGATION_TOOL, LIDAR_TOPIC)
    connect(
        NAVIGATION_TOOL,
        "loco",
        PROPOSAL_TOPIC,
        source_port="velocity_proposal",
    )

    if preserved_goal_connections:
        previous = preserved_goal_connections[0]
        source_card = next(
            (
                card
                for card in cards
                if card.get("id") == previous.get("fromCardId")
            ),
            None,
        )
        assert source_card is not None, "goal_pose source card is missing"
        source_index = int(previous.get("fromPortIdx", -1))
        source_outputs = source_card.get("topicOut") or []
        assert 0 <= source_index < len(source_outputs), "goal_pose source port is invalid"
        source_output = source_outputs[source_index]
        assert source_output.get("schema") == GOAL_SCHEMA, source_output
        goal_target_index = port_index(
            selected[NAVIGATION_TOOL]["topicIn"], port="goal_pose"
        )
        connections.append(
            {
                **previous,
                "toCardId": selected[NAVIGATION_TOOL]["id"],
                "toPortIdx": goal_target_index,
                "format": source_output.get("format", "data/json"),
                "fromTopic": previous.get("fromTopic")
                or source_output.get("topic", ""),
            }
        )

    exec_connections = [
        connection
        for connection in layout.get("execConnections") or []
        if connection.get("fromCardId") not in managed_ids
        and connection.get("toCardId") not in managed_ids
    ]
    return {
        "cards": cards,
        "connections": connections,
        "execConnections": exec_connections,
        "transform": layout.get("transform") or {},
    }


def validate(layout: dict) -> None:
    cards = layout.get("cards") or []
    connections = layout.get("connections") or []
    selected = {}
    for name in MANAGED_TOOLS:
        matches = [card for card in cards if card.get("toolName") == name]
        assert len(matches) == 1, (name, len(matches))
        selected[name] = matches[0]

    required = {
        (selected["loco_state"]["id"], selected[NAVIGATION_TOOL]["id"], STATE_TOPIC),
        (selected["lidar_cloud"]["id"], selected[NAVIGATION_TOOL]["id"], LIDAR_TOPIC),
        (selected[NAVIGATION_TOOL]["id"], selected["loco"]["id"], PROPOSAL_TOPIC),
    }
    actual = {
        (
            connection.get("fromCardId"),
            connection.get("toCardId"),
            connection.get("fromTopic"),
        )
        for connection in connections
    }
    assert required <= actual, (required, actual)
    assert sum(1 for item in actual if item in required) == 3


def save_backup(layout: dict) -> pathlib.Path:
    root = pathlib.Path("/work/resource/navigation-canvas-backups")
    root.mkdir(parents=True, exist_ok=True)
    target = root / f"before-general-navigation-{time.strftime('%Y%m%dT%H%M%S')}.json"
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps(layout, ensure_ascii=False, indent=2))
    os.replace(temporary, target)
    return target


def main() -> None:
    apply = os.environ.get("CANVAS_APPLY", "0") == "1"
    running = request("config/project-running")
    assert running == {"running": False}, running
    current = request("canvas/layout").get("data") or {}
    candidate = build_layout(current, registry_tools())
    validate(candidate)
    summary = {
        "apply": apply,
        "managed_cards": list(MANAGED_TOOLS),
        "required_connections": [STATE_TOPIC, LIDAR_TOPIC, PROPOSAL_TOPIC],
        "preserved_unrelated_cards": len(candidate["cards"]) - len(MANAGED_TOOLS),
    }
    if not apply:
        print(json.dumps(summary, ensure_ascii=False, separators=(",", ":")))
        print("GENERAL_NAVIGATION_CANVAS_WIRE_PREFLIGHT=PASS")
        return

    backup = save_backup(current)
    session_id = f"general-navigation-owner-{uuid.uuid4().hex}"
    claimed = request("canvas/claim-edit", {"session_id": session_id})
    assert claimed.get("code") == 200, claimed
    try:
        saved = request("canvas/layout", {**candidate, "session_id": session_id})
        assert saved.get("code") == 200, saved
    finally:
        request("canvas/release-edit", {"session_id": session_id})
    persisted = request("canvas/layout").get("data") or {}
    validate(persisted)
    summary["backup"] = str(backup)
    print(json.dumps(summary, ensure_ascii=False, separators=(",", ":")))
    print("GENERAL_NAVIGATION_CANVAS_WIRE=PASS")


if __name__ == "__main__":
    main()
