#!/usr/bin/env python3
import json, math, os, time, urllib.request

URL = "http://gazebo-nav:15731/mcp"
EXPECTED_LOCALIZATION_MODE = os.environ.get("EXPECTED_LOCALIZATION_MODE", "gazebo_ground_truth_odom")
EXPECTED_ODOMETRY_MODE = os.environ.get("EXPECTED_ODOMETRY_MODE", "ideal")
ACCEPTANCE_STAGE = os.environ.get("ACCEPTANCE_STAGE", "P3")
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
def rpc(method, params=None):
    body = json.dumps({"jsonrpc":"2.0","id":1,"method":method,"params":params or {}}).encode()
    value = json.loads(OPENER.open(urllib.request.Request(URL,data=body,headers={"Content-Type":"application/json"}),timeout=10).read())
    if "error" in value: raise RuntimeError(value["error"])
    return value["result"]
def call(name, action, **kwargs):
    result = rpc("tools/call", {"name":name,"arguments":{"action":action,**kwargs}})
    return json.loads(result["content"][0]["text"])

tools = {item["name"]: item for item in rpc("tools/list")["tools"]}
assert {"navigation_map","navigation"} <= tools.keys(), tools.keys()
assert tools["navigation_map"]["topic_out"][0]["format"] == "sensor/mapping"
call("navigation_map","start"); call("navigation","start")
deadline = time.time() + 90
while time.time() < deadline:
    info = call("navigation","info")
    if info["ready"]: break
    time.sleep(2)
else: raise RuntimeError(f"navigation readiness timeout: {info}")
assert info["localization_mode"] == EXPECTED_LOCALIZATION_MODE, info
assert info["odometry_mode"] == EXPECTED_ODOMETRY_MODE, info
if EXPECTED_LOCALIZATION_MODE == "amcl_laser_scan":
    assert info["amcl_lifecycle"]["label"] == "active", info
    assert info["localization_covariance"] is not None, info
    assert info["ground_truth_pose"] is not None, info
    assert info["localization_error_m"] < 0.35, info
    assert info["localization_yaw_error_rad"] < 0.35, info
try:
    call("navigation","navigate_to_pose",x=1.5,y=1.0,yaw=0.0)
    raise RuntimeError("occupied goal was accepted")
except RuntimeError as exc:
    assert "goal_occupied_or_unknown" in str(exc), exc
goal = call("navigation","navigate_to_pose",x=3.0,y=1.0,yaw=0.0)
assert goal["state"] == "navigating", goal
deadline = time.time() + 90
while time.time() < deadline:
    info = call("navigation","info")
    if info["navigation"]["state"] in {"succeeded","failed","canceled"}: break
    time.sleep(2)
assert info["navigation"]["state"] == "succeeded", info
assert info["navigation"]["error"] == "", info
assert info["navigation"]["distance_remaining"] == 0.0, info
if EXPECTED_LOCALIZATION_MODE == "amcl_laser_scan":
    assert info["localization_error_m"] < 0.35, info
    assert info["localization_yaw_error_rad"] < 0.35, info
pose = info["pose"]
assert math.hypot(pose["x"]-3.0, pose["y"]-1.0) < 0.4, pose
if EXPECTED_ODOMETRY_MODE == "deterministic_scale":
    assert info["odometry_drift_error_m"] > 0.05, info

goal = call("navigation","navigate_to_pose",x=0.0,y=0.0,yaw=0.0)
assert goal["state"] == "navigating", goal
time.sleep(1)
cancel = call("navigation","cancel")
assert cancel["state"] == "canceling" and cancel["canceled"] is True, cancel
deadline = time.time() + 15
while time.time() < deadline:
    info = call("navigation","info")
    if info["navigation"]["state"] in {"canceled","failed","succeeded"}: break
    time.sleep(0.5)
assert info["navigation"]["state"] == "canceled", info
assert info["navigation"]["error"] == "", info
assert info["navigation"]["cancel_reason"] == "user_requested", info
print(f"{ACCEPTANCE_STAGE} NAVIGATION ACCEPTANCE PASS", json.dumps(info, ensure_ascii=False))
