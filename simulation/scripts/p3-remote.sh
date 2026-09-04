#!/usr/bin/env bash
set -euo pipefail
RUNTIME_ROOT="${RUNTIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SIM_ROOT="${SIM_ROOT:-$(cd "${RUNTIME_ROOT}/.." && pwd)}"
PROJECT=phanthymotus-sim-p0
GIT_PROXY="${GIT_PROXY:-}"
CORE_IMAGE_OVERRIDE="${CORE_IMAGE_OVERRIDE:-$(docker inspect phanthymotus-sim-p0-agent-core --format '{{.Config.Image}}' 2>/dev/null || true)}"
PERCEPTION_IMAGE_OVERRIDE="${PERCEPTION_IMAGE_OVERRIDE:-$(docker inspect phanthymotus-sim-p0-perception --format '{{.Config.Image}}' 2>/dev/null || true)}"
SIM_DRIVER_IMAGE="${SIM_DRIVER_IMAGE:-$(docker inspect phanthymotus-sim-p2-g1-driver --format '{{.Config.Image}}' 2>/dev/null || true)}"
STAGE="${SIM_STAGE:-p3}"
[[ "$STAGE" == p3 || "$STAGE" == p4 || "$STAGE" == p5 ]] || { echo "SIM_STAGE must be p3, p4, or p5" >&2; exit 2; }
COMPOSE=(-f "$RUNTIME_ROOT/compose.p0.yaml" -f "$RUNTIME_ROOT/compose.p1.yaml" -f "$RUNTIME_ROOT/compose.p2.yaml" -f "$RUNTIME_ROOT/compose.p3.yaml")
SOURCE_PATHS=("$RUNTIME_ROOT/gazebo-nav" "$RUNTIME_ROOT/docker/gazebo-nav.Dockerfile" "$RUNTIME_ROOT/compose.p3.yaml" "$RUNTIME_ROOT/scripts/p3_acceptance.py")
EXPECTED_LOCALIZATION_MODE=gazebo_ground_truth_odom
EXPECTED_ODOMETRY_MODE=ideal
if [[ "$STAGE" == p4 || "$STAGE" == p5 ]]; then
  COMPOSE+=(-f "$RUNTIME_ROOT/compose.p4.yaml")
  SOURCE_PATHS+=("$RUNTIME_ROOT/compose.p4.yaml" "$RUNTIME_ROOT/scripts/p4_localization_check.py")
  EXPECTED_LOCALIZATION_MODE=amcl_laser_scan
fi
if [[ "$STAGE" == p5 ]]; then
  COMPOSE+=(-f "$RUNTIME_ROOT/compose.p5.yaml")
  SOURCE_PATHS+=("$RUNTIME_ROOT/compose.p5.yaml" "$RUNTIME_ROOT/scripts/p5_localization_check.py")
  EXPECTED_ODOMETRY_MODE=deterministic_scale
fi
source_hash(){ find "${SOURCE_PATHS[@]}" -type f ! -name '*.pyc' ! -name '*.pgm' ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -c1-12; }
image_name(){ printf 'phanthymotus-sim/gazebo-nav:%s-%s-fortress-nav2-amd64\n' "$STAGE" "$(source_hash)"; }
compose(){ CORE_IMAGE_OVERRIDE="$CORE_IMAGE_OVERRIDE" PERCEPTION_IMAGE_OVERRIDE="$PERCEPTION_IMAGE_OVERRIDE" SIM_DRIVER_IMAGE="$SIM_DRIVER_IMAGE" GAZEBO_NAV_IMAGE="$(image_name)" docker compose -p "$PROJECT" "${COMPOSE[@]}" "$@"; }
refresh_local_services(){ python3 "$RUNTIME_ROOT/scripts/render-local-services.py" --output "$RUNTIME_ROOT/state/local-services.json"; }
verify_existing_stack(){
  docker inspect \
    phanthymotus-sim-p0-agent-core \
    phanthymotus-sim-p0-perception \
    phanthymotus-sim-p2-g1-driver \
    | python3 -c '
import json, sys

expected = {
    "phanthymotus-sim-p0-agent-core",
    "phanthymotus-sim-p0-perception",
    "phanthymotus-sim-p2-g1-driver",
}
items = json.load(sys.stdin)
assert {item["Name"].lstrip("/") for item in items} == expected
for item in items:
    name = item["Name"].lstrip("/")
    host = item["HostConfig"]
    assert item["State"]["Running"] is True, name
    assert item["RestartCount"] == 0, (name, item["RestartCount"])
    assert host["Privileged"] is False, name
    assert host["NetworkMode"] == "phanthymotus-sim-p0-net", (name, host["NetworkMode"])
    assert not host.get("Devices"), (name, host.get("Devices"))
    assert not host.get("DeviceRequests"), (name, host.get("DeviceRequests"))
    assert "ROS_DOMAIN_ID=83" in item["Config"]["Env"], (name, item["Config"]["Env"])
print("P0-P2 live isolation contract PASS")
'
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
context = ssl._create_unverified_context()
response = urllib.request.urlopen("https://127.0.0.1:15678/", context=context, timeout=3)
assert response.status == 200
print("Agent Core live health PASS")
'
  docker exec phanthymotus-sim-p0-perception python3 -c '
import json, urllib.request
request = urllib.request.Request(
    "http://127.0.0.1:15720/mcp",
    data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}}).encode(),
    headers={"Content-Type": "application/json"},
)
payload = json.loads(urllib.request.urlopen(request, timeout=3).read())
assert payload["result"]["serverInfo"]["name"]
print("Perception live MCP PASS")
'
  docker exec phanthymotus-sim-p2-g1-driver python3 -c '
import json, urllib.request
health = json.loads(urllib.request.urlopen("http://127.0.0.1:15730/health", timeout=3).read())
assert health["simulation_backend"] == "mujoco_g1_29dof", health
assert health["state"]["simulation_backend"] == "mujoco_g1_29dof", health
print("MuJoCo P2 live health PASS")
'
}
preflight(){
  local map_probe
  verify_existing_stack
  python3 -m unittest discover -s "$RUNTIME_ROOT/tests" -v
  PYTHONPYCACHEPREFIX=/tmp/phanthymotus-sim-p3-pycache \
    python3 -m py_compile "$RUNTIME_ROOT/gazebo-nav/phanthymotus_sim_nav/navigation_node.py" "$RUNTIME_ROOT/gazebo-nav/launch/gazebo_nav.launch.py" "$RUNTIME_ROOT/gazebo-nav/tools/probe_drive_signs.py" "$RUNTIME_ROOT/scripts/p3_acceptance.py" "$RUNTIME_ROOT/scripts/p4_localization_check.py" "$RUNTIME_ROOT/scripts/p5_localization_check.py"
  map_probe="$(mktemp /tmp/phanthymotus-sim-p3-map.XXXXXX)"
  if ! python3 "$RUNTIME_ROOT/gazebo-nav/tools/generate_map.py" "$map_probe"; then
    rm -f "$map_probe"
    return 1
  fi
  rm -f "$map_probe"
  compose config --quiet
  printf '[%s] PREFLIGHT PASS\n' "$STAGE"
}
build(){ preflight; DOCKER_BUILDKIT=1 docker build --network host --build-arg HTTP_PROXY="$GIT_PROXY" --build-arg HTTPS_PROXY="$GIT_PROXY" --build-arg SOURCE_REVISION="$(source_hash)" -f "$RUNTIME_ROOT/docker/gazebo-nav.Dockerfile" -t "$(image_name)" "$SIM_ROOT"; }
up(){ refresh_local_services; compose up -d --no-deps gazebo-nav; refresh_local_services; }
wait_ready(){
  local deadline=$((SECONDS+120))
  until docker exec phanthymotus-sim-p3-gazebo-nav python3 -c 'import json,urllib.request; health=json.loads(urllib.request.urlopen("http://127.0.0.1:15731/health",timeout=2).read()); assert health["ready"], health' >/dev/null 2>&1; do
    ((SECONDS<deadline)) || { docker logs --tail 300 phanthymotus-sim-p3-gazebo-nav; return 1; }
    sleep 2
  done
}
verify(){
  local amcl_check_rc=0 teleport_response
  wait_ready || return $?
  docker exec phanthymotus-sim-p3-gazebo-nav bash -lc 'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /sim_ws/install/setup.bash && python3 /sim_ws/src/phanthymotus_sim_nav/tools/probe_drive_signs.py' || return $?
  if [[ "$STAGE" == p3 ]]; then
    docker exec phanthymotus-sim-p3-gazebo-nav python3 -c 'import json,math,time,urllib.request; time.sleep(1); health=json.loads(urllib.request.urlopen("http://127.0.0.1:15731/health",timeout=2).read()); pose=health["pose"]; truth=health["ground_truth_pose"]; normalize=lambda p:(p["x"],p["y"],math.atan2(math.sin(p["yaw"]),math.cos(p["yaw"]))); values=[normalize(pose),normalize(truth)]; assert all(abs(x)<0.12 and abs(y)<0.12 and abs(yaw)<0.12 for x,y,yaw in values), values; print("P3 probe pose restoration PASS",values)' || return $?
  fi
  docker cp "$RUNTIME_ROOT/scripts/p3_acceptance.py" phanthymotus-sim-p0-agent-core:/tmp/p3_acceptance.py || return $?
  docker exec -e ACCEPTANCE_STAGE="${STAGE^^}" -e EXPECTED_LOCALIZATION_MODE="$EXPECTED_LOCALIZATION_MODE" -e EXPECTED_ODOMETRY_MODE="$EXPECTED_ODOMETRY_MODE" phanthymotus-sim-p0-agent-core .venv/bin/python /tmp/p3_acceptance.py || return $?
  if [[ "$STAGE" == p4 || "$STAGE" == p5 ]]; then
    docker cp "$RUNTIME_ROOT/scripts/p4_localization_check.py" phanthymotus-sim-p0-agent-core:/tmp/p4_localization_check.py || return $?
    docker exec phanthymotus-sim-p3-gazebo-nav bash -lc 'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /sim_ws/install/setup.bash && ros2 lifecycle set /amcl deactivate' || return $?
    docker exec phanthymotus-sim-p0-agent-core .venv/bin/python /tmp/p4_localization_check.py unavailable || amcl_check_rc=$?
    docker exec phanthymotus-sim-p3-gazebo-nav bash -lc 'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /sim_ws/install/setup.bash && [[ "$(ros2 lifecycle get /amcl)" == active\ \[* ]] || ros2 lifecycle set /amcl activate' || amcl_check_rc=$?
    ((amcl_check_rc == 0)) || return "$amcl_check_rc"
    docker exec phanthymotus-sim-p0-agent-core .venv/bin/python /tmp/p4_localization_check.py ready || return $?
  fi
  if [[ "$STAGE" == p5 ]]; then
    teleport_response="$(docker exec phanthymotus-sim-p3-gazebo-nav ign service -s /world/synthetic_room/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 5000 --req 'name: "planar_base" position {x: 0.0 y: 2.5 z: 0.15} orientation {z: -0.47942554 w: 0.87758256}')" || return $?
    grep -q 'data: true' <<<"$teleport_response" || { printf 'Gazebo teleport failed: %s\n' "$teleport_response" >&2; return 1; }
    docker cp "$RUNTIME_ROOT/scripts/p5_localization_check.py" phanthymotus-sim-p0-agent-core:/tmp/p5_localization_check.py || return $?
    docker exec phanthymotus-sim-p0-agent-core .venv/bin/python /tmp/p5_localization_check.py || return $?
  fi
  docker inspect phanthymotus-sim-p3-gazebo-nav | python3 -c 'import json,sys; x=json.load(sys.stdin)[0]; h=x["HostConfig"]; assert x["State"]["Running"] and x["RestartCount"]==0; assert not h["Privileged"] and not h.get("Devices") and h["NetworkMode"]=="phanthymotus-sim-p0-net"; print("Gazebo Navigation isolation PASS")' || return $?
}
deploy_and_verify(){
  local previous_image previous_mode previous_odometry_mode rc
  previous_image="$(docker inspect -f '{{.Config.Image}}' phanthymotus-sim-p3-gazebo-nav 2>/dev/null || true)"
  previous_mode="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' phanthymotus-sim-p3-gazebo-nav 2>/dev/null | awk -F= '$1 == "LOCALIZATION_MODE" {print $2; exit}' || true)"
  previous_odometry_mode="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' phanthymotus-sim-p3-gazebo-nav 2>/dev/null | awk -F= '$1 == "ODOMETRY_MODE" {print $2; exit}' || true)"
  build
  up
  if verify; then
    printf '[%s] DEPLOY + VERIFY PASS\n' "$STAGE"
  else
    rc=$?
    if [[ -n "$previous_image" ]]; then
      local -a restore_compose=(-f "$RUNTIME_ROOT/compose.p0.yaml" -f "$RUNTIME_ROOT/compose.p1.yaml" -f "$RUNTIME_ROOT/compose.p2.yaml" -f "$RUNTIME_ROOT/compose.p3.yaml")
      [[ "$previous_mode" == amcl ]] && restore_compose+=(-f "$RUNTIME_ROOT/compose.p4.yaml")
      [[ "$previous_odometry_mode" == deterministic_scale ]] && restore_compose+=(-f "$RUNTIME_ROOT/compose.p5.yaml")
      if GAZEBO_NAV_IMAGE="$previous_image" docker compose -p "$PROJECT" "${restore_compose[@]}" up -d --no-deps gazebo-nav; then
        printf '[%s] verification failed; restored Gazebo Navigation image=%s mode=%s\n' "$STAGE" "$previous_image" "${previous_mode:-ground_truth}" >&2
      else
        printf '[%s] verification failed; ROLLBACK FAILED image=%s\n' "$STAGE" "$previous_image" >&2
      fi
    else
      compose rm -sf gazebo-nav || true
      printf '[%s] verification failed; new Gazebo Navigation service removed; P0-P2 kept running\n' "$STAGE" >&2
    fi
    refresh_local_services || true
    return "$rc"
  fi
}
case "${1:-}" in preflight) preflight;; build) build;; up) up;; verify) verify;; deploy-and-verify) deploy_and_verify;; *) echo "usage: $0 {preflight|build|up|verify|deploy-and-verify}" >&2; exit 2;; esac
