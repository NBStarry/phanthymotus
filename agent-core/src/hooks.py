"""
System Hooks — bypass-LLM immediate actions triggered by system events.

Hooks are registered by drivers via `x-hooks` in MCP tool schemas.
When fired, they execute tool calls directly (no LLM, no barrier, no ACP).
"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

# ── Types ────────────────────────────────────────────────────────────────────


@dataclass
class HookBinding:
    mcp_id: str
    tool: str
    action: str
    params: dict = field(default_factory=dict)


# ── Registry ─────────────────────────────────────────────────────────────────

_registry: dict[str, list[HookBinding]] = {}
_fire_log: deque = deque(maxlen=30)
_speech_gate: asyncio.Future | None = None
_speech_gate_timer: asyncio.Task | None = None


def open_speech_gate(failsafe_s: float = 35.0) -> None:
    """Hold new TTS calls until ASR yields a command or the window expires."""
    global _speech_gate, _speech_gate_timer
    if _speech_gate is not None and not _speech_gate.done():
        _speech_gate.set_result('reopened')
    if _speech_gate_timer is not None:
        _speech_gate_timer.cancel()

    loop = asyncio.get_running_loop()
    future = loop.create_future()
    _speech_gate = future

    async def _failsafe():
        global _speech_gate, _speech_gate_timer
        try:
            await asyncio.sleep(failsafe_s)
        except asyncio.CancelledError:
            return
        if _speech_gate is future and not future.done():
            future.set_result('failsafe')
            _speech_gate = None
            _speech_gate_timer = None

    _speech_gate_timer = asyncio.create_task(_failsafe())


def release_speech_gate(reason: str, clear: bool = False) -> bool:
    """Release current TTS waiters; optionally forget the completed window."""
    global _speech_gate, _speech_gate_timer
    future = _speech_gate
    if future is None:
        return False
    if not future.done():
        future.set_result(reason)
    if _speech_gate_timer is not None:
        _speech_gate_timer.cancel()
        _speech_gate_timer = None
    if clear:
        _speech_gate = None
    return True


def clear_speech_gate() -> None:
    """Forget a completed window before processing the new user command."""
    global _speech_gate, _speech_gate_timer
    if _speech_gate_timer is not None:
        _speech_gate_timer.cancel()
        _speech_gate_timer = None
    _speech_gate = None


async def wait_speech_gate() -> str:
    """Return the release reason, or ``inactive`` when no window is open."""
    future = _speech_gate
    if future is None:
        return 'inactive'
    return await asyncio.shield(future)


def register(mcp_id: str, tool_name: str, x_hooks: dict):
    """Register hook bindings from a tool's x-hooks schema field.

    Args:
        mcp_id: MCP device ID (e.g. "mcp-1785810828")
        tool_name: Tool name (e.g. "smart_motion")
        x_hooks: Dict of hook_id → {"action": str, "params": dict}
    """
    for hook_id, spec in x_hooks.items():
        action = spec.get('action', '')
        params = spec.get('params', {})
        binding = HookBinding(mcp_id=mcp_id, tool=tool_name, action=action, params=params)
        if hook_id not in _registry:
            _registry[hook_id] = []
        # Avoid duplicate bindings (same mcp_id + tool + action)
        existing = [(b.mcp_id, b.tool, b.action) for b in _registry[hook_id]]
        if (mcp_id, tool_name, action) not in existing:
            _registry[hook_id].append(binding)
            print(f'[hooks] registered: {hook_id} → {mcp_id}/{tool_name}.{action}')


def unregister_device(mcp_id: str):
    """Remove all bindings for a device (e.g. when it goes offline)."""
    for hook_id in list(_registry.keys()):
        _registry[hook_id] = [b for b in _registry[hook_id] if b.mcp_id != mcp_id]
        if not _registry[hook_id]:
            del _registry[hook_id]


def list_hooks() -> dict[str, list[dict]]:
    """Return all registered hooks and their bindings."""
    return {
        hook_id: [
            {'mcp_id': b.mcp_id, 'tool': b.tool, 'action': b.action, 'params': b.params}
            for b in bindings
        ]
        for hook_id, bindings in _registry.items()
    }


def is_interrupt_binding(mcp_id: str, tool: str, action: str) -> bool:
    """Check if tool+action is registered under an on_interrupt_* hook.
    Used by barrier logic to exempt interrupt actions from blocking."""
    for hook_id, bindings in _registry.items():
        if not hook_id.startswith('on_interrupt'):
            continue
        for b in bindings:
            if b.mcp_id == mcp_id and b.tool == tool and b.action == action:
                return True
    return False


def get_hook_for_binding(mcp_id: str, tool: str, action: str) -> str | None:
    """Find the hook_id that a tool+action is registered under."""
    for hook_id, bindings in _registry.items():
        for b in bindings:
            if b.mcp_id == mcp_id and b.tool == tool and b.action == action:
                return hook_id
    return None


def get_status() -> dict:
    """Return hook registry and recent fire log for diagnostics."""
    return {
        'registry': list_hooks(),
        'recent_fires': list(_fire_log),
    }


# ── Executor ─────────────────────────────────────────────────────────────────

async def fire(hook_id: str, extra_params: dict | None = None, exclude_mcp_id: str | None = None) -> list[dict]:
    """Fire a hook: execute all bound tool calls immediately, bypassing LLM and barrier.

    Args:
        hook_id: Hook identifier (e.g. "on_interrupt_all")
        extra_params: Additional params merged into each tool call
        exclude_mcp_id: Skip bindings from this mcp_id (avoid double-fire)

    Returns:
        List of results from each binding execution.
    """
    import mcp_client  # deferred to avoid circular import

    bindings = _registry.get(hook_id, [])
    if exclude_mcp_id:
        bindings = [b for b in bindings if b.mcp_id != exclude_mcp_id]
    if not bindings:
        return []

    results = []
    tasks = []

    for binding in bindings:
        args = {**binding.params, **(extra_params or {})}
        if binding.action:
            args['action'] = binding.action
        tasks.append(_fire_one(mcp_client, binding, args))

    # Execute all bindings in parallel
    outcomes = await asyncio.gather(*tasks, return_exceptions=True)
    for binding, outcome in zip(bindings, outcomes):
        if isinstance(outcome, Exception):
            results.append({'hook': hook_id, 'mcp_id': binding.mcp_id,
                           'tool': binding.tool, 'error': str(outcome)})
            print(f'[hooks] {hook_id} → {binding.tool}.{binding.action} FAILED: {outcome}')
        else:
            results.append({'hook': hook_id, 'mcp_id': binding.mcp_id,
                           'tool': binding.tool, 'result': outcome})

    if bindings:
        print(f'[hooks] fired {hook_id}: {len(bindings)} binding(s)')
    _fire_log.append({
        'hook': hook_id, 'ts': time.time(),
        'bindings': len(bindings),
        'errors': [r for r in results if 'error' in r],
    })
    return results


async def _fire_one(mcp_client, binding: HookBinding, args: dict) -> Any:
    """Execute a single hook binding via direct tool call."""
    return await mcp_client.call_tool_direct(binding.mcp_id, binding.tool, args)
