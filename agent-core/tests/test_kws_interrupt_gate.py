"""Speech-gate lifecycle for KWS barge-in; no MCP or ROS runtime required."""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path


CORE_SRC = Path(__file__).resolve().parents[1] / "src"
if str(CORE_SRC) not in sys.path:
    sys.path.insert(0, str(CORE_SRC))

import hooks  # noqa: E402
import collector  # noqa: E402
import event_bus  # noqa: E402


def test_command_suppresses_stale_speech_but_timeout_releases_it():
    async def run():
        hooks.clear_speech_gate()
        hooks.open_speech_gate(failsafe_s=1)
        command_waiter = asyncio.create_task(hooks.wait_speech_gate())
        await asyncio.sleep(0)
        assert hooks.release_speech_gate("command")
        assert await command_waiter == "command"
        assert await hooks.wait_speech_gate() == "command"

        hooks.clear_speech_gate()
        hooks.open_speech_gate(failsafe_s=1)
        timeout_waiter = asyncio.create_task(hooks.wait_speech_gate())
        await asyncio.sleep(0)
        assert hooks.release_speech_gate("timeout", clear=True)
        assert await timeout_waiter == "timeout"
        assert await hooks.wait_speech_gate() == "inactive"

    asyncio.run(run())


def test_kws_interrupt_steers_even_when_global_mode_is_interrupt():
    async def run():
        event_bus._queue = asyncio.Queue()
        collector._steering_queue = asyncio.Queue()
        collector._output = asyncio.Queue()
        collector._priority_pending.clear()
        collector._source_ring.clear()
        collector._busy = True
        collector._interrupt_mode = "interrupt"
        collector._cancel_event = asyncio.Event()
        await event_bus.enqueue(
            source="dds:/mic/asr",
            text='{"text":"过来","priority":1,"audio_duration_ms":800,'
                 '"kws_triggered":true,"kws_interrupt":true}',
        )
        await event_bus.enqueue(
            source="asr:kws_interrupt_timeout",
            text='{"type":"kws_interrupt_timeout","priority":1,'
                 '"text":"continue prior action"}',
        )

        task = asyncio.create_task(collector._drain_loop())
        try:
            event = await asyncio.wait_for(
                collector._steering_queue.get(), timeout=1)
            assert event["_kws_interrupt"]
            timeout_event = await asyncio.wait_for(
                collector._steering_queue.get(), timeout=1)
            assert timeout_event["_kws_interrupt_timeout"]
            assert not collector._cancel_event.is_set()
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            collector._busy = False
            collector._interrupt_mode = "steer"
            collector._cancel_event = None

    asyncio.run(run())
