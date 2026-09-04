#!/usr/bin/env python3
import json
import sys
import time
import urllib.request

URL = "http://gazebo-nav:15731/mcp"
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def call(action, **kwargs):
    body = json.dumps({
        "jsonrpc": "2.0",
        "id": 1,
        "method": "tools/call",
        "params": {"name": "navigation", "arguments": {"action": action, **kwargs}},
    }).encode()
    value = json.loads(OPENER.open(urllib.request.Request(
        URL, data=body, headers={"Content-Type": "application/json"}
    ), timeout=10).read())
    if "error" in value:
        raise RuntimeError(value["error"])
    return json.loads(value["result"]["content"][0]["text"])


expected = sys.argv[1]
deadline = time.time() + 30
while time.time() < deadline:
    info = call("info")
    if expected == "ready" and info["ready"]:
        break
    if expected == "unavailable" and not info["localization_ready"]:
        break
    time.sleep(0.5)
else:
    raise RuntimeError(f"localization did not become {expected}: {info}")

assert info["localization_mode"] == "amcl_laser_scan", info
if expected == "ready":
    assert info["amcl_lifecycle"]["label"] == "active", info
    print("P4 AMCL RECOVERY PASS")
elif expected == "unavailable":
    assert info["ready"] is False, info
    call("start")
    try:
        call("navigate_to_pose", x=0.0, y=0.0, yaw=0.0)
    except RuntimeError as exc:
        assert "navigation_not_ready" in str(exc), exc
    else:
        raise RuntimeError("navigation accepted a goal while AMCL was unavailable")
    print("P4 AMCL FAILURE SIGNAL PASS")
else:
    raise ValueError("usage: p4_localization_check.py {ready|unavailable}")
