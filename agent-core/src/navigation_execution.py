"""Trusted canvas control plane for navigation-to-Driver execution leases.

The navigation Perception card only publishes bounded velocity proposals.  The
G1 Driver actuator will consume them only after Agent Core binds the exact
``nav_id`` for one task.  This module owns that binding lifecycle so neither
the Perception card nor an LLM can arm the Driver directly.
"""

from __future__ import annotations

import asyncio
import json
import uuid
from dataclasses import dataclass
from typing import Any, Awaitable, Callable


RawInvoker = Callable[[str, str, dict], Awaitable[Any]]


class ExecutionControlError(RuntimeError):
    """Stable fail-closed control-plane error."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class ExecutionLink:
    source_card_id: str
    source_mcp_id: str
    source_tool: str
    target_card_id: str
    target_mcp_id: str
    target_tool: str
    proposal_topic: str
    proposal_schema: str


@dataclass(frozen=True)
class ExecutionLease:
    nav_id: str
    link: ExecutionLink


_leases: dict[tuple[str, str], ExecutionLease] = {}
_locks: dict[tuple[str, str], asyncio.Lock] = {}


def _tool_definition(mcp_entries: list, mcp_id: str, tool_name: str) -> dict:
    entry = next((item for item in mcp_entries if item.get("id") == mcp_id), None)
    if not entry:
        raise ExecutionControlError(
            "execution_mcp_not_registered", f"MCP {mcp_id} is not registered"
        )
    tool = next(
        (
            item
            for item in entry.get("tools", [])
            if isinstance(item, dict) and item.get("name") == tool_name
        ),
        None,
    )
    if not tool:
        raise ExecutionControlError(
            "execution_tool_not_registered",
            f"tool {tool_name} is not registered on MCP {mcp_id}",
        )
    return tool


def _ports(card: dict, tool: dict, key: str) -> list:
    card_key = "topicOut" if key == "topic_out" else "topicIn"
    value = card.get(card_key) or tool.get(key) or []
    return [item for item in value if isinstance(item, dict)]


def _port_matches(port: dict, *, name: str, schema: str, topic: str = "") -> bool:
    if name and port.get("port") != name:
        return False
    if schema and port.get("schema") != schema:
        return False
    if topic and port.get("topic") != topic:
        return False
    return True


def resolve_execution_link(
    *,
    layout: dict,
    mcp_entries: list,
    source_mcp_id: str,
    source_tool: str,
    control: dict,
) -> ExecutionLink:
    """Resolve and validate one proposal wire from navigation to one actuator."""

    cards = layout.get("cards", [])
    connections = layout.get("connections", [])
    source_cards = [
        card
        for card in cards
        if card.get("mcpId") == source_mcp_id
        and card.get("toolName") == source_tool
    ]
    if len(source_cards) != 1:
        raise ExecutionControlError(
            "execution_source_card_invalid",
            f"expected exactly one {source_tool} canvas card, got {len(source_cards)}",
        )
    source_card = source_cards[0]
    source_def = _tool_definition(mcp_entries, source_mcp_id, source_tool)

    output_port = str(control.get("output_port", "velocity_proposal"))
    proposal_schema = str(control.get("proposal_schema", "")).strip()
    expected_target_tool = str(control.get("target_tool", "")).strip()
    if not proposal_schema:
        raise ExecutionControlError(
            "execution_contract_invalid", "proposal_schema is required"
        )

    source_ports = _ports(source_card, source_def, "topic_out")
    source_indexes = [
        index
        for index, port in enumerate(source_ports)
        if _port_matches(port, name=output_port, schema=proposal_schema)
    ]
    if len(source_indexes) != 1:
        raise ExecutionControlError(
            "execution_output_port_invalid",
            f"expected exactly one {output_port}/{proposal_schema} output",
        )
    source_index = source_indexes[0]
    proposal_topic = str(source_ports[source_index].get("topic", "")).strip()
    if not proposal_topic:
        raise ExecutionControlError(
            "execution_output_topic_missing", "velocity proposal topic is empty"
        )

    matching_connections = []
    for connection in connections:
        if connection.get("fromCardId") != source_card.get("id"):
            continue
        try:
            port_index = int(connection.get("fromPortIdx", -1))
        except (TypeError, ValueError):
            continue
        if port_index != source_index:
            continue
        persisted_topic = str(connection.get("fromTopic", "")).strip()
        if persisted_topic and persisted_topic != proposal_topic:
            continue
        matching_connections.append(connection)

    if len(matching_connections) != 1:
        raise ExecutionControlError(
            "execution_driver_connection_invalid",
            "connect the navigation velocity_proposal output to exactly one Driver actuator",
        )

    connection = matching_connections[0]
    target_card = next(
        (card for card in cards if card.get("id") == connection.get("toCardId")),
        None,
    )
    if not target_card:
        raise ExecutionControlError(
            "execution_target_card_missing", "Driver actuator card is missing"
        )
    target_mcp_id = str(target_card.get("mcpId", ""))
    target_tool = str(target_card.get("toolName", ""))
    if expected_target_tool and target_tool != expected_target_tool:
        raise ExecutionControlError(
            "execution_target_tool_invalid",
            f"expected Driver tool {expected_target_tool}, got {target_tool or 'empty'}",
        )
    target_def = _tool_definition(mcp_entries, target_mcp_id, target_tool)
    if target_def.get("type") != "actuator":
        raise ExecutionControlError(
            "execution_target_not_actuator",
            f"target tool {target_tool} is not an actuator",
        )

    target_ports = _ports(target_card, target_def, "topic_in")
    try:
        target_index = int(connection.get("toPortIdx", -1))
    except (TypeError, ValueError):
        target_index = -1
    if target_index < 0 or target_index >= len(target_ports):
        raise ExecutionControlError(
            "execution_input_port_invalid", "Driver input port index is invalid"
        )
    if not _port_matches(
        target_ports[target_index],
        name=output_port,
        schema=proposal_schema,
        topic=proposal_topic,
    ):
        raise ExecutionControlError(
            "execution_input_contract_mismatch",
            "Driver input must match the navigation velocity proposal topic and schema",
        )

    return ExecutionLink(
        source_card_id=str(source_card.get("id", "")),
        source_mcp_id=source_mcp_id,
        source_tool=source_tool,
        target_card_id=str(target_card.get("id", "")),
        target_mcp_id=target_mcp_id,
        target_tool=target_tool,
        proposal_topic=proposal_topic,
        proposal_schema=proposal_schema,
    )


def discover_execution_links(*, layout: dict, mcp_entries: list) -> list[ExecutionLink]:
    """Resolve every execution-controlled source present on the canvas."""

    links = []
    seen_sources: set[tuple[str, str]] = set()
    for card in layout.get("cards", []):
        source_mcp_id = str(card.get("mcpId", ""))
        source_tool = str(card.get("toolName", ""))
        key = (source_mcp_id, source_tool)
        if not all(key) or key in seen_sources:
            continue
        seen_sources.add(key)
        tool = _tool_definition(mcp_entries, source_mcp_id, source_tool)
        control = tool.get("x-execution-control")
        if not isinstance(control, dict):
            continue
        links.append(
            resolve_execution_link(
                layout=layout,
                mcp_entries=mcp_entries,
                source_mcp_id=source_mcp_id,
                source_tool=source_tool,
                control=control,
            )
        )
    return links


def _parse_result(raw: Any) -> dict | None:
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, list) and raw:
        first = raw[0]
        if isinstance(first, dict) and first.get("type") == "text":
            raw = first.get("text", "")
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None
    return None


def _encode_result(value: dict) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def _error(action: str, code: str, message: str, **details: Any) -> str:
    return _encode_result(
        {
            "action": action,
            "status": "error",
            "error_code": code,
            "error": message,
            **details,
        }
    )


def _snapshot() -> tuple[dict, list]:
    import config

    return (
        config.main.get("canvas_layout", {}),
        config.main.get("services", {}).get("mcp", []),
    )


def _managed_actions(control: dict) -> set[str]:
    result: set[str] = set()
    for key in (
        "start_actions",
        "wait_actions",
        "stop_actions",
        "pause_actions",
        "resume_actions",
    ):
        result.update(str(item) for item in control.get(key, []) if item)
    return result


async def _stop_driver(lease: ExecutionLease, invoke: RawInvoker) -> tuple[bool, Any]:
    raw = await invoke(
        lease.link.target_mcp_id,
        lease.link.target_tool,
        {
            "action": "stop",
            "instance_id": lease.link.target_card_id,
        },
    )
    parsed = _parse_result(raw)
    ok = bool(
        parsed
        and parsed.get("connected") is False
        and parsed.get("stop_confirmed") is True
        and parsed.get("state") == "idle"
    )
    return ok, parsed if parsed is not None else raw


async def _release_current(
    key: tuple[str, str], lease: ExecutionLease, invoke: RawInvoker
) -> tuple[bool, Any]:
    ok, result = await _stop_driver(lease, invoke)
    lock = _locks.setdefault(key, asyncio.Lock())
    async with lock:
        if _leases.get(key) == lease and ok:
            _leases.pop(key, None)
    return ok, result


async def call_with_execution_lease(
    *,
    source_mcp_id: str,
    source_tool: str,
    action: str,
    arguments: dict,
    control: dict,
    invoke: RawInvoker,
) -> Any:
    """Invoke one navigation action under the canvas-bound Driver lease."""

    if action not in _managed_actions(control):
        return await invoke(source_mcp_id, source_tool, arguments)

    key = (source_mcp_id, source_tool)
    lock = _locks.setdefault(key, asyncio.Lock())
    start_actions = set(control.get("start_actions", []))
    wait_actions = set(control.get("wait_actions", []))
    stop_actions = set(control.get("stop_actions", []))
    pause_actions = set(control.get("pause_actions", []))
    resume_actions = set(control.get("resume_actions", []))
    terminal_statuses = set(control.get("terminal_statuses", []))
    lease_argument = str(control.get("lease_argument", "_control_nav_id"))

    if action in start_actions:
        async with lock:
            if key in _leases:
                return _error(
                    action,
                    "execution_lease_active",
                    f"navigation {_leases[key].nav_id} still owns the Driver lease",
                )
            try:
                layout, mcp_entries = _snapshot()
                link = resolve_execution_link(
                    layout=layout,
                    mcp_entries=mcp_entries,
                    source_mcp_id=source_mcp_id,
                    source_tool=source_tool,
                    control=control,
                )
            except ExecutionControlError as exc:
                return _error(action, exc.code, str(exc))

            nav_id = uuid.uuid4().hex
            driver_raw = await invoke(
                link.target_mcp_id,
                link.target_tool,
                {
                    "action": "start",
                    "instance_id": link.target_card_id,
                    "input_topic": link.proposal_topic,
                    "expected_nav_id": nav_id,
                },
            )
            driver = _parse_result(driver_raw)
            if not (
                driver
                and driver.get("state") == "ready"
                and driver.get("connected") is True
                and driver.get("armed") is True
                and driver.get("expected_nav_id") == nav_id
            ):
                return _error(
                    action,
                    "execution_driver_bind_failed",
                    "Driver did not acknowledge the trusted navigation lease",
                    driver_result=driver if driver is not None else driver_raw,
                )
            lease = ExecutionLease(nav_id=nav_id, link=link)
            _leases[key] = lease

        try:
            raw = await invoke(
                source_mcp_id,
                source_tool,
                {**arguments, lease_argument: nav_id},
            )
        except Exception:
            await _release_current(key, lease, invoke)
            raise
        result = _parse_result(raw)
        if not result or result.get("status") == "error":
            stop_ok, stop_result = await _release_current(key, lease, invoke)
            if not stop_ok:
                return _error(
                    action,
                    "execution_driver_stop_unconfirmed",
                    "navigation failed and Driver stop was not confirmed",
                    navigation_result=result if result is not None else raw,
                    driver_result=stop_result,
                )
            return raw
        result["execution"] = {
            "driver_mcp_id": link.target_mcp_id,
            "driver_tool": link.target_tool,
            "expected_nav_id": nav_id,
            "state": "armed",
        }
        if result.get("status") in terminal_statuses:
            stop_ok, stop_result = await _release_current(key, lease, invoke)
            if not stop_ok:
                return _error(
                    action,
                    "execution_driver_stop_unconfirmed",
                    "Driver stop was not confirmed after terminal navigation",
                    navigation_result=result,
                    driver_result=stop_result,
                )
            result["execution"]["state"] = "released"
            result["execution"]["stop_confirmed"] = True
        return _encode_result(result)

    lease = _leases.get(key)
    if action in resume_actions and lease is None:
        return _error(
            action,
            "execution_lease_retired",
            "resume requires a live Driver lease; start a new navigation task",
        )
    if action in wait_actions | pause_actions and lease is None:
        return _error(
            action,
            "execution_lease_missing",
            f"{action} requires a live Driver navigation lease",
        )

    raw = await invoke(source_mcp_id, source_tool, arguments)
    result = _parse_result(raw)
    should_release = bool(
        lease
        and (
            action in stop_actions
            or action in pause_actions
            or (result and result.get("status") in terminal_statuses)
        )
    )
    if not should_release:
        return raw

    stop_ok, stop_result = await _release_current(key, lease, invoke)
    if not stop_ok:
        return _error(
            action,
            "execution_driver_stop_unconfirmed",
            "Driver stop was not confirmed",
            navigation_result=result if result is not None else raw,
            driver_result=stop_result,
        )
    if result is None:
        return raw
    result["execution"] = {
        "driver_mcp_id": lease.link.target_mcp_id,
        "driver_tool": lease.link.target_tool,
        "expected_nav_id": lease.nav_id,
        "state": "released",
        "stop_confirmed": True,
    }
    return _encode_result(result)


def reset_state_for_tests() -> None:
    """Clear in-memory leases; intentionally not used by production code."""

    _leases.clear()
    _locks.clear()


def clear_lease_for_source(source_mcp_id: str, source_tool: str) -> None:
    """Forget a lease only after the project lifecycle confirmed Driver stop."""

    _leases.pop((source_mcp_id, source_tool), None)
