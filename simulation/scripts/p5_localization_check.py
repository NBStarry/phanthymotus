#!/usr/bin/env python3
import json
import math
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


deadline = time.time() + 10
while time.time() < deadline:
    before = call("info")
    if before["ground_truth_age_s"] < 1 and before["localization_error_m"] > 1.0:
        break
    time.sleep(0.5)
else:
    raise RuntimeError(f"kidnapped pose was not observed: {before}")

result = call("relocalize")
assert result["state"] == "relocalizing", result
initial_yaw = before["ground_truth_pose"]["yaw"]
scan_seen = False
max_scan_rotation = 0.0
try:
    call("navigate_to_pose", x=0.0, y=0.0, yaw=0.0)
except RuntimeError as exc:
    assert "navigation_not_ready" in str(exc), exc
else:
    raise RuntimeError("navigation accepted a goal during relocalization")

deadline = time.time() + 75
while time.time() < deadline:
    info = call("info")
    scan_seen = scan_seen or info["relocalization"]["scan_active"]
    truth_yaw = info["ground_truth_pose"]["yaw"]
    max_scan_rotation = max(
        max_scan_rotation,
        abs(math.atan2(math.sin(truth_yaw - initial_yaw), math.cos(truth_yaw - initial_yaw))),
    )
    if info["ready"] and info["localization_error_m"] < 0.35:
        break
    time.sleep(1)
else:
    raise RuntimeError(f"global relocalization did not converge: {info}")

assert scan_seen, info
assert max_scan_rotation > 2.5, (max_scan_rotation, info)
assert info["odometry_mode"] == "deterministic_scale", info
assert info["odometry_drift_error_m"] > 1.0, info
goal = call("navigate_to_pose", x=-3.0, y=1.5, yaw=0.0)
assert goal["state"] == "navigating", goal
deadline = time.time() + 90
while time.time() < deadline:
    info = call("info")
    if info["navigation"]["state"] in {"succeeded", "failed", "canceled"}:
        break
    time.sleep(1)
assert info["navigation"]["state"] == "succeeded", info
truth = info["ground_truth_pose"]
assert math.hypot(truth["x"] + 3.0, truth["y"] - 1.5) < 0.45, info
assert info["localization_error_m"] < 0.35, info
print("P5 KIDNAPPED ROBOT RECOVERY PASS", json.dumps(info, ensure_ascii=False))
