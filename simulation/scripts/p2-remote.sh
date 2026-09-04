#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="${RUNTIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SIM_ROOT="${SIM_ROOT:-$(cd "${RUNTIME_ROOT}/.." && pwd)}"
BASE_COMPOSE="${RUNTIME_ROOT}/compose.p0.yaml"
P1_COMPOSE="${RUNTIME_ROOT}/compose.p1.yaml"
P2_COMPOSE="${RUNTIME_ROOT}/compose.p2.yaml"
PROJECT="phanthymotus-sim-p0"
GIT_PROXY="${GIT_PROXY:-}"
BUILD_NO_PROXY="localhost,127.0.0.1,.4pd.io,172.17.0.0/16,172.28.0.0/16"
UNITREE_MUJOCO_ROOT="${RUNTIME_ROOT}/src/unitree_mujoco"
UNITREE_MUJOCO_REVISION="ae6a8403e272733e9996ef59990880330496177f"
PILLOW_WHEEL="${RUNTIME_ROOT}/artifacts/python-wheels/pillow-11.3.0-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl"
PILLOW_SHA256="4445fa62e15936a028672fd48c4c11a66d641d2c05726c7ec1f8ba6a572036ae"
P2_WHEELS="${RUNTIME_ROOT}/artifacts/python-wheels-p2"
CORE_IMAGE_OVERRIDE="${CORE_IMAGE_OVERRIDE:-$(docker inspect phanthymotus-sim-p0-agent-core --format '{{.Config.Image}}' 2>/dev/null || true)}"
PERCEPTION_IMAGE_OVERRIDE="${PERCEPTION_IMAGE_OVERRIDE:-$(docker inspect phanthymotus-sim-p0-perception --format '{{.Config.Image}}' 2>/dev/null || true)}"

log() {
  printf '[phanthymotus-sim-p2] %s\n' "$*"
}

refresh_local_services() {
  python3 "$RUNTIME_ROOT/scripts/render-local-services.py" \
    --output "$RUNTIME_ROOT/state/local-services.json"
}

prepare_context() {
  :
}

source_hash() {
  {
    find "$RUNTIME_ROOT/sim-driver" \
      "$RUNTIME_ROOT/tests" \
      "$RUNTIME_ROOT/docker/sim-driver-p2.Dockerfile" \
      "$RUNTIME_ROOT/compose.p2.yaml" \
      -type f -print0 \
      | sort -z \
      | xargs -0 sha256sum
    printf '%s\n' "$UNITREE_MUJOCO_REVISION"
  } | sha256sum | cut -c1-12
}

image_name() {
  printf 'phanthymotus-sim/sim-driver:p2-%s-mujoco-amd64\n' "$(source_hash)"
}

compose() {
  CORE_IMAGE_OVERRIDE="$CORE_IMAGE_OVERRIDE" \
    PERCEPTION_IMAGE_OVERRIDE="$PERCEPTION_IMAGE_OVERRIDE" \
    SIM_DRIVER_IMAGE="$(image_name)" \
    docker compose -p "$PROJECT" \
      -f "$BASE_COMPOSE" -f "$P1_COMPOSE" -f "$P2_COMPOSE" "$@"
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

verify_p2_wheels() {
  test -d "$P2_WHEELS"
  sha256sum -c <<EOF
0f17b89f2a4eaaedc4f28c622998aa690564b3012a396a4ffad0821007fe03ba  ${P2_WHEELS}/absl_py-2.5.0-py3-none-any.whl
d9cd4f40fbe77ad6613b7348a18132cc511237b6c076dbb89105c0b520a4c6bb  ${P2_WHEELS}/etils-1.13.0-py3-none-any.whl
b57ddbafedfaef7018c1ecab32aa200a9d7ca26b77965f64e48b70061249d279  ${P2_WHEELS}/fsspec-2026.7.0-py3-none-any.whl
b860de3ca0686182483f98f3ddd12e660acf25b3e0d521450ec9a3f999f72a65  ${P2_WHEELS}/glfw-2.10.2-py2.py3-none-manylinux_2_28_x86_64.whl
1bd7b48b4088eddb2cd16382150bb515af0bd2c70128194392725f82ad2c96a1  ${P2_WHEELS}/importlib_resources-7.1.0-py3-none-any.whl
7640bc40229fce2611be76999b04cc6e8fadd315a40165ddeab48764ac5c2878  ${P2_WHEELS}/mujoco-3.3.6-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
794a943daced39300879e4e47bd94525280685f42dbb5a998d336cfff151d74f  ${P2_WHEELS}/pyopengl-3.1.10-py3-none-any.whl
f0fa19c6845758ab08074a0cfa8b7aecb71c999ca73d62883bc25cc018c4e548  ${P2_WHEELS}/typing_extensions-4.15.0-py3-none-any.whl
071652d6115ed432f5ce1d34c336c0adfd6a884660d1e9712a256d3d3bd4b14e  ${P2_WHEELS}/zipp-3.23.0-py3-none-any.whl
EOF
}

preflight() {
  bash "$RUNTIME_ROOT/scripts/p0-remote.sh" preflight
  prepare_context
  log 'checking locked official G1 model, remote-downloaded wheels and local tests'
  test "$(git -C "$UNITREE_MUJOCO_ROOT" rev-parse HEAD)" = "$UNITREE_MUJOCO_REVISION"
  test -z "$(git -C "$UNITREE_MUJOCO_ROOT" status --short)"
  test -f "$UNITREE_MUJOCO_ROOT/LICENSE"
  test -f "$UNITREE_MUJOCO_ROOT/unitree_robots/g1/scene_29dof.xml"
  test -f "$PILLOW_WHEEL"
  printf '%s  %s\n' "$PILLOW_SHA256" "$PILLOW_WHEEL" | sha256sum -c -
  verify_p2_wheels
  python3 -m unittest discover -s "$RUNTIME_ROOT/tests" -v
  python3 -m py_compile \
    "$RUNTIME_ROOT/sim-driver/main.py" \
    "$RUNTIME_ROOT/sim-driver/mujoco_backend.py" \
    "$RUNTIME_ROOT/scripts/p2_acceptance.py" \
    "$RUNTIME_ROOT/scripts/p2_demo.py"
  compose config --quiet
  if ! docker inspect phanthymotus-sim-p1-g1-driver >/dev/null 2>&1 \
    && ! docker inspect phanthymotus-sim-p2-g1-driver >/dev/null 2>&1; then
    check_port_free 16730
  fi
  log 'PREFLIGHT PASS'
}

build() {
  preflight
  local image revision
  image="$(image_name)"
  revision="$(source_hash)"
  log "building ${image}; model, wheels and build context remain on wlcb-23"
  DOCKER_BUILDKIT=1 docker build --network host \
    --build-arg HTTP_PROXY="$GIT_PROXY" --build-arg HTTPS_PROXY="$GIT_PROXY" \
    --build-arg NO_PROXY="$BUILD_NO_PROXY" --build-arg no_proxy="$BUILD_NO_PROXY" \
    --build-arg SOURCE_REVISION="$revision" \
    --build-arg UNITREE_MUJOCO_REVISION="$UNITREE_MUJOCO_REVISION" \
    -f "$RUNTIME_ROOT/docker/sim-driver-p2.Dockerfile" \
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
    if docker exec phanthymotus-sim-p2-g1-driver python3 -c '
import json, urllib.request
health=json.loads(urllib.request.urlopen("http://127.0.0.1:15730/health",timeout=3).read())
assert health["simulation_backend"] == "mujoco_g1_29dof"
assert health["state"]["simulation_backend"] == "mujoco_g1_29dof"
' >/dev/null 2>&1; then
      break
    fi
    sleep 2
  done
  if (( SECONDS >= deadline )); then
    printf 'MuJoCo Sim Driver readiness timeout after 90s\n' >&2
    compose ps >&2 || true
    docker logs --tail 200 phanthymotus-sim-p2-g1-driver >&2 || true
    return 1
  fi

  docker cp "$RUNTIME_ROOT/scripts/p2_acceptance.py" \
    phanthymotus-sim-p0-agent-core:/tmp/p2_acceptance.py
  docker exec phanthymotus-sim-p0-agent-core bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && .venv/bin/python /tmp/p2_acceptance.py'

  docker inspect \
    phanthymotus-sim-p0-agent-core \
    phanthymotus-sim-p0-perception \
    phanthymotus-sim-p2-g1-driver \
    | python3 -c '
import json, sys
items = json.load(sys.stdin)
expected = {
  "phanthymotus-sim-p0-agent-core": (2_000_000_000, 2 * 1024**3),
  "phanthymotus-sim-p0-perception": (4_000_000_000, 8 * 1024**3),
  "phanthymotus-sim-p2-g1-driver": (2_000_000_000, 2 * 1024**3),
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
print("P2 isolation + 8 CPU / 12 GiB G1 budget PASS")
'
  docker image inspect "$(image_name)" -f '{{ index .Config.Labels "phanthymotus.sim.unitree-mujoco-revision" }}' \
    | grep -Fx "$UNITREE_MUJOCO_REVISION" >/dev/null
  compose ps
  log 'VERIFY PASS'
}

rollback() {
  local previous_image="$1"
  local previous_stage="$2"
  if [[ -z "$previous_image" ]]; then
    log 'no previous Sim Driver image recorded; automatic rollback skipped'
    return 0
  fi
  log "restoring previous Sim Driver image=${previous_image}"
  if [[ "$previous_stage" == "p2" ]]; then
    SIM_DRIVER_IMAGE="$previous_image" \
      docker compose -p "$PROJECT" \
        -f "$BASE_COMPOSE" -f "$P1_COMPOSE" -f "$P2_COMPOSE" up -d --no-deps sim-driver
    docker inspect phanthymotus-sim-p2-g1-driver -f '{{.State.Running}} {{.Config.Image}}'
  else
    SIM_DRIVER_IMAGE="$previous_image" \
      docker compose -p "$PROJECT" -f "$BASE_COMPOSE" -f "$P1_COMPOSE" up -d --no-deps sim-driver
    docker inspect phanthymotus-sim-p1-g1-driver -f '{{.State.Running}} {{.Config.Image}}'
  fi
  refresh_local_services
}

deploy_and_verify() {
  local previous_image=''
  local previous_stage='p1'
  if docker inspect phanthymotus-sim-p1-g1-driver >/dev/null 2>&1; then
    previous_image="$(docker inspect phanthymotus-sim-p1-g1-driver -f '{{.Config.Image}}')"
  elif docker inspect phanthymotus-sim-p2-g1-driver >/dev/null 2>&1; then
    previous_image="$(docker inspect phanthymotus-sim-p2-g1-driver -f '{{.Config.Image}}')"
    previous_stage='p2'
  fi
  build
  up
  if verify; then
    log 'DEPLOY + VERIFY PASS'
  else
    local rc=$?
    rollback "$previous_image" "$previous_stage" || true
    return "$rc"
  fi
}

demo() {
  local action="${1:-}"
  case "$action" in
    wave|fall|reset|stop) ;;
    *)
      printf 'usage: %s demo {wave|fall|reset|stop}\n' "$0" >&2
      return 2
      ;;
  esac
  docker cp "$RUNTIME_ROOT/scripts/p2_demo.py" \
    phanthymotus-sim-p0-agent-core:/tmp/p2_demo.py
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python \
    /tmp/p2_demo.py "$action"
}

case "${1:-}" in
  preflight) preflight ;;
  build) build ;;
  up) up ;;
  verify) verify ;;
  deploy-and-verify) deploy_and_verify ;;
  demo) demo "${2:-}" ;;
  *)
    printf 'usage: %s {preflight|build|up|verify|deploy-and-verify|demo ACTION}\n' "$0" >&2
    exit 2
    ;;
esac
