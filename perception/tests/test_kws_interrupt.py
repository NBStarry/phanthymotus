"""Host-side contracts for the dedicated KWS interruption mode."""

from __future__ import annotations

import queue
import sys
import threading
from pathlib import Path


PERCEPTION_ROOT = Path(__file__).resolve().parents[1]
if str(PERCEPTION_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_ROOT))

from plugins import asr, tts  # noqa: E402


def test_kws_interrupt_schema_and_window_contract():
    schema = asr.TOOLS[0]["configSchema"]["properties"]
    assert "kws_interrupt" in schema["trigger_mode"]["enum"]
    assert schema["kws_model"]["x-show-when"]["trigger_mode"] == [
        "kws", "kws_interrupt"]
    assert not asr._kws_interrupt_timed_out(10.0, False, 14.999)
    assert asr._kws_interrupt_timed_out(10.0, False, 15.0)
    assert not asr._kws_interrupt_timed_out(10.0, True, 99.0)


def test_idle_tts_interrupt_does_not_cancel_the_next_utterance():
    node = object.__new__(tts._TTSNode)
    node._text_queue = queue.Queue()
    node._speaking = threading.Event()
    node._interrupt_flag = threading.Event()

    node.interrupt()
    assert not node._interrupt_flag.is_set()
    node._speaking.set()
    node.interrupt()
    assert node._interrupt_flag.is_set()
