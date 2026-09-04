#!/usr/bin/env python3
"""Trigger deterministic P2 motions while watching the Phanthy Motus WebUI."""

from __future__ import annotations

import json
import sys
import time
from urllib import request as urllib_request


MCP_URL = "http://sim-driver:15730/mcp"


def call(name: str, arguments: dict) -> dict:
    payload = json.dumps(
        {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {"name": name, "arguments": arguments},
        }
    ).encode()
    req = urllib_request.Request(
        MCP_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
    )
    with urllib_request.urlopen(req, timeout=5) as response:
        rpc = json.loads(response.read())
    if "error" in rpc:
        raise RuntimeError(f"{name} failed: {rpc['error']}")
    return json.loads(rpc["result"]["content"][0]["text"])


def main() -> None:
    action = sys.argv[1] if len(sys.argv) > 1 else ""
    if action not in {"wave", "fall", "reset", "stop"}:
        raise SystemExit("usage: p2_demo.py {wave|fall|reset|stop}")

    call("joints", {"action": "start"})
    call("loco_state", {"action": "start"})
    if action == "wave":
        call("gesture", {"action": "start"})
        result = call("gesture", {"action": "wave", "duration": 6.0})
    elif action == "fall":
        call("sim_control", {"action": "start"})
        call("sim_control", {"action": "reset", "seed": 23})
        time.sleep(2.0)
        call("sim_control", {"action": "set_balance_assist", "enabled": False})
        result = call(
            "sim_control",
            {"action": "push", "fx": 400.0, "fy": 0.0, "duration": 0.3},
        )
    elif action == "reset":
        call("sim_control", {"action": "start"})
        result = call("sim_control", {"action": "reset", "seed": 23})
    else:
        call("gesture", {"action": "start"})
        call("gesture", {"action": "stop_wave"})
        call("joints", {"action": "stop"})
        call("loco_state", {"action": "stop"})
        result = {"state": "idle", "streams_stopped": ["joints", "loco_state"]}

    print(json.dumps({"demo": action, "result": result}, ensure_ascii=False))


if __name__ == "__main__":
    main()
