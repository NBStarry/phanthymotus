"""Route declared JSON topic inputs through ordinary trusted MCP tool calls."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import json
from typing import Any


class TopicActionError(RuntimeError):
    pass


@dataclass(frozen=True)
class TopicActionRoute:
    card_id: str
    mcp_id: str
    tool: str
    port: str
    topic: str
    schema: str
    action: str
    wait_action: str
    stop_action: str
    id_field: str
    allowed_fields: tuple[str, ...]

    @property
    def subscription_id(self) -> str:
        return f"topic-action:{self.card_id}:{self.port}"


def _tool_definition(mcp_entries: list, mcp_id: str, tool_name: str) -> dict:
    entry = next((item for item in mcp_entries if item.get("id") == mcp_id), None)
    if not entry:
        raise TopicActionError(f"MCP {mcp_id} is not registered")
    tool = next(
        (
            item
            for item in entry.get("tools", [])
            if isinstance(item, dict) and item.get("name") == tool_name
        ),
        None,
    )
    if not tool:
        raise TopicActionError(f"tool {tool_name} is not registered on {mcp_id}")
    return tool


def _ports(card: dict, tool: dict, key: str) -> list[dict]:
    card_key = "topicIn" if key == "topic_in" else "topicOut"
    value = card.get(card_key) or tool.get(key) or []
    return [item for item in value if isinstance(item, dict)]


def resolve_topic_action_routes(*, layout: dict, mcp_entries: list) -> list[TopicActionRoute]:
    routes: list[TopicActionRoute] = []
    cards = layout.get("cards", [])
    connections = layout.get("connections", [])
    for card in cards:
        mcp_id = str(card.get("mcpId", ""))
        tool_name = str(card.get("toolName", ""))
        if not mcp_id or not tool_name:
            continue
        tool = _tool_definition(mcp_entries, mcp_id, tool_name)
        declarations = tool.get("x-topic-actions")
        if not isinstance(declarations, list):
            continue
        inputs = _ports(card, tool, "topic_in")
        for declaration in declarations:
            if not isinstance(declaration, dict):
                raise TopicActionError("x-topic-actions entries must be objects")
            port_name = str(declaration.get("port", "")).strip()
            schema = str(declaration.get("schema", "")).strip()
            action = str(declaration.get("action", "")).strip()
            wait_action = str(declaration.get("wait_action", "")).strip()
            stop_action = str(declaration.get("stop_action", "")).strip()
            id_field = str(declaration.get("id_field", "goal_id")).strip()
            allowed_fields = tuple(
                str(item) for item in declaration.get("allowed_fields", []) if item
            )
            indexes = [
                index for index, port in enumerate(inputs)
                if port.get("port") == port_name and port.get("schema") == schema
            ]
            if (
                len(indexes) != 1
                or not action
                or not wait_action
                or not stop_action
                or not id_field
                or not allowed_fields
            ):
                raise TopicActionError(
                    f"invalid topic action declaration for {tool_name}.{port_name}"
                )
            target_index = indexes[0]
            matches = []
            for connection in connections:
                if connection.get("toCardId") != card.get("id"):
                    continue
                try:
                    connection_index = int(connection.get("toPortIdx", -1))
                except (TypeError, ValueError):
                    continue
                if connection_index == target_index:
                    matches.append(connection)
            if not matches:
                continue
            if len(matches) != 1:
                raise TopicActionError(
                    f"connect exactly one source to {tool_name}.{port_name}"
                )
            connection = matches[0]
            source_card = next(
                (item for item in cards if item.get("id") == connection.get("fromCardId")),
                None,
            )
            if not source_card:
                raise TopicActionError(f"source card for {tool_name}.{port_name} is missing")
            source_tool = _tool_definition(
                mcp_entries,
                str(source_card.get("mcpId", "")),
                str(source_card.get("toolName", "")),
            )
            outputs = _ports(source_card, source_tool, "topic_out")
            try:
                output_index = int(connection.get("fromPortIdx", -1))
            except (TypeError, ValueError):
                output_index = -1
            if output_index < 0 or output_index >= len(outputs):
                raise TopicActionError(f"source port for {tool_name}.{port_name} is invalid")
            output = outputs[output_index]
            topic = str(connection.get("fromTopic") or output.get("topic") or "").strip()
            if not topic or output.get("schema") != schema:
                raise TopicActionError(
                    f"source for {tool_name}.{port_name} must publish {schema}"
                )
            routes.append(
                TopicActionRoute(
                    card_id=str(card.get("id", "")),
                    mcp_id=mcp_id,
                    tool=tool_name,
                    port=port_name,
                    topic=topic,
                    schema=schema,
                    action=action,
                    wait_action=wait_action,
                    stop_action=stop_action,
                    id_field=id_field,
                    allowed_fields=allowed_fields,
                )
            )
    return routes


_active_routes: dict[str, TopicActionRoute] = {}
_seen_ids: dict[str, list[str]] = {}


def _decode_goal(route: TopicActionRoute, data: bytes) -> tuple[str, dict]:
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TopicActionError(f"invalid JSON: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("schema") != route.schema:
        raise TopicActionError(f"message schema must be {route.schema}")
    goal_id = payload.get(route.id_field)
    if not isinstance(goal_id, str) or not goal_id.strip() or len(goal_id) > 128:
        raise TopicActionError(f"{route.id_field} must be a non-empty string")
    allowed = {"schema", route.id_field, *route.allowed_fields}
    unexpected = sorted(set(payload) - allowed)
    if unexpected:
        raise TopicActionError("unexpected fields: " + ",".join(unexpected))
    arguments = {
        "action": route.action,
        **{name: payload[name] for name in route.allowed_fields if name in payload},
    }
    return goal_id.strip(), arguments


def _result_payload(result: dict) -> dict:
    if not isinstance(result, dict) or result.get("code") != 200:
        raise TopicActionError(f"MCP call failed: {result}")
    data = result.get("data")
    if isinstance(data, list) and data:
        first = data[0]
        if isinstance(first, dict):
            try:
                data = json.loads(first.get("text", "{}"))
            except (TypeError, json.JSONDecodeError) as exc:
                raise TopicActionError(f"invalid MCP result: {exc}") from exc
    elif isinstance(data, str):
        try:
            data = json.loads(data)
        except json.JSONDecodeError as exc:
            raise TopicActionError(f"invalid MCP result: {exc}") from exc
    if not isinstance(data, dict):
        raise TopicActionError(f"MCP result is not an object: {data}")
    return data


async def _handle_message(route: TopicActionRoute, data: bytes) -> None:
    from api.mcp_manage import MCPCallRequest, mcp_call_tool
    from api.motus_stream import push_event
    import config

    try:
        if config.main.get("core", {}).get("project_running") is not True:
            raise TopicActionError("canvas project is not running")
        goal_id, arguments = _decode_goal(route, data)
        seen = _seen_ids.setdefault(route.subscription_id, [])
        if goal_id in seen:
            raise TopicActionError(f"duplicate {route.id_field}: {goal_id}")
        seen.append(goal_id)
        del seen[:-256]
        start_result = await mcp_call_tool(
            route.mcp_id,
            MCPCallRequest(tool=route.tool, arguments=arguments),
        )
        start_payload = _result_payload(start_result)
        if start_payload.get("status") == "error" or start_payload.get("state") == "error":
            terminal_result = None
        else:
            nav_id = start_payload.get("nav_id")
            if not isinstance(nav_id, str) or not nav_id:
                raise TopicActionError("navigation start did not return nav_id")
            try:
                terminal_result = await mcp_call_tool(
                    route.mcp_id,
                    MCPCallRequest(
                        tool=route.tool,
                        arguments={
                            "action": route.wait_action,
                            "stall_timeout": 90.0,
                        },
                    ),
                )
                _result_payload(terminal_result)
            except Exception:
                await mcp_call_tool(
                    route.mcp_id,
                    MCPCallRequest(
                        tool=route.tool,
                        arguments={"action": route.stop_action},
                    ),
                )
                raise
        await push_event({
            "type": "topic_action_result",
            "payload": {
                "card_id": route.card_id,
                "port": route.port,
                "topic": route.topic,
                "goal_id": goal_id,
                "start_result": start_result,
                "terminal_result": terminal_result,
            },
        })
    except Exception as exc:
        await push_event({
            "type": "topic_action_error",
            "payload": {
                "card_id": route.card_id,
                "port": route.port,
                "topic": route.topic,
                "error": f"{type(exc).__name__}: {exc}",
            },
        })


async def start_topic_action_routes(*, layout: dict, mcp_entries: list) -> list[TopicActionRoute]:
    import ros2_bridge

    stop_topic_action_routes()
    routes = resolve_topic_action_routes(layout=layout, mcp_entries=mcp_entries)
    loop = asyncio.get_running_loop()
    for route in routes:
        async def callback(data: bytes, _fmt: str, selected=route) -> None:
            await _handle_message(selected, data)

        subscribed = ros2_bridge.subscribe(
            route.subscription_id,
            route.topic,
            "data/json",
            loop,
            callback,
            reliable=True,
        )
        if subscribed is not True:
            stop_topic_action_routes()
            raise TopicActionError(
                f"ROS 2 bridge could not subscribe to {route.topic}"
            )
        _active_routes[route.subscription_id] = route
    return routes


def stop_topic_action_routes() -> None:
    import ros2_bridge

    for subscription_id in list(_active_routes):
        ros2_bridge.unsubscribe(subscription_id)
    _active_routes.clear()
    _seen_ids.clear()
