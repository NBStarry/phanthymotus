#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="${RUNTIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SIM_ROOT="${SIM_ROOT:-$(cd "${RUNTIME_ROOT}/.." && pwd)}"
BASE_COMPOSE="${RUNTIME_ROOT}/compose.p0.yaml"
P1_COMPOSE="${RUNTIME_ROOT}/compose.p1.yaml"
PROJECT="phanthymotus-sim-p0"
GIT_PROXY="${GIT_PROXY:-}"
BUILD_NO_PROXY="localhost,127.0.0.1,.4pd.io,172.17.0.0/16,172.28.0.0/16"
PILLOW_WHEEL="${RUNTIME_ROOT}/artifacts/python-wheels/pillow-11.3.0-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl"
PILLOW_SHA256="4445fa62e15936a028672fd48c4c11a66d641d2c05726c7ec1f8ba6a572036ae"
CORE_IMAGE_OVERRIDE="${CORE_IMAGE_OVERRIDE:-$(docker inspect phanthymotus-sim-p0-agent-core --format '{{.Config.Image}}' 2>/dev/null || true)}"
PERCEPTION_IMAGE_OVERRIDE="${PERCEPTION_IMAGE_OVERRIDE:-$(docker inspect phanthymotus-sim-p0-perception --format '{{.Config.Image}}' 2>/dev/null || true)}"

log() {
  printf '[phanthymotus-sim-p1] %s\n' "$*"
}

refresh_local_services() {
  python3 "$RUNTIME_ROOT/scripts/render-local-services.py" \
    --output "$RUNTIME_ROOT/state/local-services.json"
}

source_hash() {
  find "$RUNTIME_ROOT/sim-driver" "$RUNTIME_ROOT/docker/sim-driver.Dockerfile" \
    -type f -print0 \
    | sort -z \
    | xargs -0 sha256sum \
    | sha256sum \
    | cut -c1-12
}

image_name() {
  printf 'phanthymotus-sim/sim-driver:p1-%s-amd64\n' "$(source_hash)"
}

compose() {
  CORE_IMAGE_OVERRIDE="$CORE_IMAGE_OVERRIDE" \
    PERCEPTION_IMAGE_OVERRIDE="$PERCEPTION_IMAGE_OVERRIDE" \
    SIM_DRIVER_IMAGE="$(image_name)" \
    docker compose -p "$PROJECT" -f "$BASE_COMPOSE" -f "$P1_COMPOSE" "$@"
}

check_port_free() {
  python3 - "$1" <<'PY'
import socket
import sys

port = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.bind(("127.0.0.1", port))
except OSError as exc:
    raise SystemExit(f"host port is already in use: {port}: {exc}")
finally:
    sock.close()
PY
}

preflight() {
  bash "$RUNTIME_ROOT/scripts/p0-remote.sh" preflight
  log 'checking remote-downloaded Pillow artifact and local P1 tests'
  test -f "$PILLOW_WHEEL"
  printf '%s  %s\n' "$PILLOW_SHA256" "$PILLOW_WHEEL" | sha256sum -c -
  python3 -m unittest discover -s "$RUNTIME_ROOT/tests" -v
  compose config --quiet
  if ! docker inspect phanthymotus-sim-p1-g1-driver >/dev/null 2>&1; then
    check_port_free 16730
  fi
  log 'PREFLIGHT PASS'
}

build() {
  preflight
  local image revision
  image="$(image_name)"
  revision="$(source_hash)"
  log "building ${image}; dependencies and build context remain on wlcb-23"
  DOCKER_BUILDKIT=1 docker build --network host \
    --build-arg HTTP_PROXY="$GIT_PROXY" --build-arg HTTPS_PROXY="$GIT_PROXY" \
    --build-arg NO_PROXY="$BUILD_NO_PROXY" --build-arg no_proxy="$BUILD_NO_PROXY" \
    --build-arg SOURCE_REVISION="$revision" \
    -f "$RUNTIME_ROOT/docker/sim-driver.Dockerfile" \
    -t "$image" "$SIM_ROOT"
  docker image inspect "$image" >/dev/null
  log "BUILD PASS image=${image}"
}

up() {
  refresh_local_services
  compose up -d --no-deps sim-driver
  refresh_local_services
  log "services started image=$(image_name)"
}

verify() {
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if docker exec phanthymotus-sim-p1-g1-driver python3 -c '
import json, urllib.request
data=json.dumps({"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}).encode()
req=urllib.request.Request("http://127.0.0.1:15730/mcp",data=data,headers={"Content-Type":"application/json"})
payload=json.loads(urllib.request.urlopen(req,timeout=3).read())
assert payload["result"]["serverInfo"]["name"] == "sim-g1-device-bundle"
' >/dev/null 2>&1; then
      break
    fi
    sleep 2
  done
  if (( SECONDS >= deadline )); then
    printf 'sim driver readiness timeout after 90s\n' >&2
    compose ps >&2 || true
    docker logs --tail 160 phanthymotus-sim-p1-g1-driver >&2 || true
    return 1
  fi

  docker cp "$RUNTIME_ROOT/scripts/p1_acceptance.py" \
    phanthymotus-sim-p0-agent-core:/tmp/p1_acceptance.py
  docker exec phanthymotus-sim-p0-agent-core bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && .venv/bin/python /tmp/p1_acceptance.py'

  docker inspect \
    phanthymotus-sim-p0-agent-core \
    phanthymotus-sim-p0-perception \
    phanthymotus-sim-p1-g1-driver \
    | python3 -c '
import json, sys
items = json.load(sys.stdin)
expected = {
  "phanthymotus-sim-p0-agent-core": (2_000_000_000, 2 * 1024**3),
  "phanthymotus-sim-p0-perception": (4_000_000_000, 8 * 1024**3),
  "phanthymotus-sim-p1-g1-driver": (2_000_000_000, 2 * 1024**3),
}
for item in items:
    name = item["Name"].lstrip("/")
    host = item["HostConfig"]
    assert item["State"]["Running"] is True, name
    assert item["RestartCount"] == 0, (name, item["RestartCount"])
    assert host["Privileged"] is False, name
    assert host["NetworkMode"] == "phanthymotus-sim-p0-net", (name, host["NetworkMode"])
    assert not host.get("Devices"), (name, host.get("Devices"))
    assert not host.get("DeviceRequests"), (name, host.get("DeviceRequests"))
    assert host["NanoCpus"] == expected[name][0], (name, host["NanoCpus"])
    assert host["Memory"] == expected[name][1], (name, host["Memory"])
    assert "ROS_DOMAIN_ID=83" in item["Config"]["Env"], item["Config"]["Env"]
print("P1 isolation + 8 CPU / 12 GiB G1 budget PASS")
'
  compose ps
  log 'VERIFY PASS'
}

case "${1:-}" in
  preflight) preflight ;;
  build) build ;;
  up) up ;;
  verify) verify ;;
  deploy-and-verify) build; up; verify ;;
  *)
    printf 'usage: %s {preflight|build|up|verify|deploy-and-verify}\n' "$0" >&2
    exit 2
    ;;
esac
