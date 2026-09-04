#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="${RUNTIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SIM_ROOT="${SIM_ROOT:-$(cd "${RUNTIME_ROOT}/.." && pwd)}"
PHANTHYMOTUS_SRC="${PHANTHYMOTUS_SRC:-${SIM_ROOT}}"
DRIVER_SRC="${DRIVER_SRC:-${RUNTIME_ROOT}/src/phanthymotus-driver}"
COMPOSE_FILE="${RUNTIME_ROOT}/compose.p0.yaml"
LOCAL_SERVICES_FILE="${RUNTIME_ROOT}/state/local-services.json"
PROJECT="phanthymotus-sim-p0"
GIT_PROXY="${GIT_PROXY:-}"
RUNTIME_PROXY="${PHANTHY_SIM_RUNTIME_PROXY:-}"
LLM_PROBE_URL="${PHANTHY_SIM_LLM_PROBE_URL:-https://router.phanthy.com/v1/models}"
BUILD_NO_PROXY="localhost,127.0.0.1,.4pd.io,172.17.0.0/16,172.28.0.0/16"

PHANTHYMOTUS_REF="refs/heads/sim"
PHANTHYMOTUS_REMOTE_MIRROR="https://ghfast.top/https://github.com/NBStarry/phanthymotus.git"
DRIVER_SHA="a9511ebc06db1276f66aa773aceb5b9e2f066279"

source_head() {
  git -C "$PHANTHYMOTUS_SRC" rev-parse HEAD 2>/dev/null && return
  local owner_uid owner_gid
  owner_uid="$(stat -c '%u' "$PHANTHYMOTUS_SRC")"
  owner_gid="$(stat -c '%g' "$PHANTHYMOTUS_SRC")"
  HOME=/tmp XDG_CONFIG_HOME=/tmp \
    setpriv --reuid="$owner_uid" --regid="$owner_gid" --clear-groups \
    git -C "$PHANTHYMOTUS_SRC" rev-parse HEAD
}

PHANTHYMOTUS_SHA="${PHANTHYMOTUS_SHA:-$(source_head)}"
PHANTHYMOTUS_SHORT="${PHANTHYMOTUS_SHA:0:12}"
CORE_IMAGE="phanthymotus-sim/agent-core:${PHANTHYMOTUS_SHORT}-amd64"
PERCEPTION_IMAGE="phanthymotus-sim/perception:${PHANTHYMOTUS_SHORT}-amd64"
ROS_BOOTSTRAP_IMAGE="local/phanthy-motus/ros-base:humble-amd64-c124798-v3"
ROS_BOOTSTRAP_ID="sha256:e21ebc229c42057682115596e6f390784946c7e8c1843e78ee94a8526e9cfe9d"

log() {
  printf '[phanthymotus-sim-p0] %s\n' "$*"
}

compose() {
  CORE_IMAGE_OVERRIDE="${CORE_IMAGE_OVERRIDE:-$CORE_IMAGE}" \
    PERCEPTION_IMAGE_OVERRIDE="${PERCEPTION_IMAGE_OVERRIDE:-$PERCEPTION_IMAGE}" \
    docker compose -p "$PROJECT" -f "$COMPOSE_FILE" "$@"
}

ensure_local_services_file() {
  mkdir -p "$(dirname "$LOCAL_SERVICES_FILE")"
  if [[ ! -f "$LOCAL_SERVICES_FILE" ]]; then
    printf '[]\n' >"$LOCAL_SERVICES_FILE"
    chmod 0644 "$LOCAL_SERVICES_FILE"
  fi
}

render_local_services() {
  ensure_local_services_file
  python3 "$RUNTIME_ROOT/scripts/render-local-services.py" \
    --output "$LOCAL_SERVICES_FILE"
}

require_exact_source() {
  local repo="$1"
  local expected="$2"
  local actual owner_uid owner_gid
  # JuiceFS 上的源码归属远端登录用户，而构建在 hzb_dev 容器内以 root 运行。
  # 只把两条只读 Git 检查切换为源码实际 UID/GID，不修改全局 safe.directory。
  owner_uid="$(stat -c '%u' "$repo")"
  owner_gid="$(stat -c '%g' "$repo")"
  actual="$(HOME=/tmp XDG_CONFIG_HOME=/tmp \
    setpriv --reuid="$owner_uid" --regid="$owner_gid" --clear-groups \
    git -C "$repo" rev-parse HEAD)"
  if [[ "$actual" != "$expected" ]]; then
    printf 'source mismatch: repo=%s expected=%s actual=%s\n' "$repo" "$expected" "$actual" >&2
    return 1
  fi
  if [[ -n "$(HOME=/tmp XDG_CONFIG_HOME=/tmp \
    setpriv --reuid="$owner_uid" --regid="$owner_gid" --clear-groups \
    git -C "$repo" status --porcelain)" ]]; then
    printf 'source is dirty: %s\n' "$repo" >&2
    return 1
  fi
}

check_port_free() {
  local port="$1"
  python3 - "$port" <<'PY'
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

lock_contract() {
  log 'checking version lock, Compose images and patch-free build contract'
  python3 - \
    "$RUNTIME_ROOT/versions.lock.yaml" \
    "$RUNTIME_ROOT/docker/agent-core.Dockerfile" \
    "$COMPOSE_FILE" \
    "$PHANTHYMOTUS_SHA" \
    "$PHANTHYMOTUS_REF" \
    "$CORE_IMAGE" <<'PY'
import pathlib
import sys

import yaml

lock_path, dockerfile_path, compose_path, revision, ref, core_image = sys.argv[1:]
lock = yaml.safe_load(pathlib.Path(lock_path).read_text())
source = lock['sources']['phanthymotus']
assert ref == f"refs/heads/{source['branch']}", (ref, source['branch'])
assert source['upstream'].endswith('/NBStarry/phanthymotus.git'), source['upstream']
assert source['upstream_base']['commit'], source

dockerfile = pathlib.Path(dockerfile_path).read_text()
assert 'COPY agent-core/' in dockerfile
assert 'runtime/patches/' not in dockerfile
assert 'CORE_PATCH_REVISION' not in dockerfile

compose = yaml.safe_load(pathlib.Path(compose_path).read_text())
assert compose['services']['agent-core']['image'] == '${CORE_IMAGE_OVERRIDE}'
assert compose['services']['perception']['image'] == '${PERCEPTION_IMAGE_OVERRIDE}'
print('version lock + patch-free build contract PASS')
PY
}

preflight() {
  lock_contract
  log 'checking exact source locks'
  require_exact_source "$PHANTHYMOTUS_SRC" "$PHANTHYMOTUS_SHA"
  require_exact_source "$DRIVER_SRC" "$DRIVER_SHA"
  local ros_bootstrap_actual
  ros_bootstrap_actual="$(docker image inspect "$ROS_BOOTSTRAP_IMAGE" --format '{{.Id}}')"
  if [[ "$ros_bootstrap_actual" != "$ROS_BOOTSTRAP_ID" ]]; then
    printf 'ROS bootstrap mismatch: expected=%s actual=%s\n' "$ROS_BOOTSTRAP_ID" "$ros_bootstrap_actual" >&2
    return 1
  fi

  log 'checking GitHub mirror through current proxy'
  git -c http.proxy="$GIT_PROXY" -c https.proxy="$GIT_PROXY" \
    ls-remote "$PHANTHYMOTUS_REMOTE_MIRROR" "$PHANTHYMOTUS_REF" \
    | grep -F "$PHANTHYMOTUS_SHA"

  log 'checking isolated host ports'
  local running
  running="$(compose ps -q 2>/dev/null || true)"
  if [[ -z "$running" ]]; then
    check_port_free 16678
    check_port_free 16720
    check_port_free 16721
  fi

  log 'checking Compose model and resource budget'
  compose config --quiet
  python3 - "$RUNTIME_ROOT/resource-profiles/g1-orin-nx-16gb.yaml" <<'PY'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))
services = p['services']
assert sum(v['cpus'] for v in services.values()) == p['policy']['total_application_cpus']
assert sum(v['memory_bytes'] for v in services.values()) == p['policy']['total_application_memory_bytes']
assert p['policy']['gpu_access'] == 'disabled'
print('resource profile PASS')
PY
  log 'PREFLIGHT PASS'
}

build_core_image() {
  log 'building Agent Core at locked sim branch revision'
  DOCKER_BUILDKIT=1 docker build --network host \
    --build-arg HTTP_PROXY="$GIT_PROXY" --build-arg HTTPS_PROXY="$GIT_PROXY" \
    --build-arg NO_PROXY="$BUILD_NO_PROXY" --build-arg no_proxy="$BUILD_NO_PROXY" \
    --build-arg SOURCE_REVISION="$PHANTHYMOTUS_SHORT" \
    -f "$RUNTIME_ROOT/docker/agent-core.Dockerfile" \
    -t "$CORE_IMAGE" "$PHANTHYMOTUS_SRC"
}

build_core() {
  preflight
  build_core_image
  docker image inspect "$CORE_IMAGE" >/dev/null
  log "CORE BUILD PASS image=${CORE_IMAGE}"
}

build() {
  preflight
  log 'building x86 ROS base from verified wlcb-23 cache; all new dependencies stay remote'
  DOCKER_BUILDKIT=1 docker build --network host \
    --build-arg HTTP_PROXY="$GIT_PROXY" --build-arg HTTPS_PROXY="$GIT_PROXY" \
    --build-arg NO_PROXY="$BUILD_NO_PROXY" --build-arg no_proxy="$BUILD_NO_PROXY" \
    -f "$RUNTIME_ROOT/docker/ros-base.Dockerfile" \
    -t phanthymotus-sim/ros-base:humble-amd64 "$PHANTHYMOTUS_SRC"

  build_core_image

  log 'building CPU-only Perception P0 runtime'
  DOCKER_BUILDKIT=1 docker build --network host \
    --build-arg HTTP_PROXY="$GIT_PROXY" --build-arg HTTPS_PROXY="$GIT_PROXY" \
    --build-arg NO_PROXY="$BUILD_NO_PROXY" --build-arg no_proxy="$BUILD_NO_PROXY" \
    -f "$RUNTIME_ROOT/docker/perception.Dockerfile" \
    -t "$PERCEPTION_IMAGE" "$PHANTHYMOTUS_SRC"

  docker image inspect \
    phanthymotus-sim/ros-base:humble-amd64 \
    "$CORE_IMAGE" \
    "$PERCEPTION_IMAGE" >/dev/null
  log 'BUILD PASS'
}

up() {
  ensure_local_services_file
  compose up -d
  render_local_services
  log 'services started; use verify for acceptance'
}

verify() {
  render_local_services
  local ready=0
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    # 脚本在 hzb_dev 中运行，而 127.0.0.1 发布端口属于 wlcb-23 宿主网络命名空间。
    # 因此从各服务容器内部验证实际 API，不把 hzb_dev 的 localhost 误当宿主。
    if docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
ctx = ssl._create_unverified_context()
print(urllib.request.urlopen("https://127.0.0.1:15678/", context=ctx, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-web.html 2>/dev/null && \
       docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
ctx = ssl._create_unverified_context()
print(urllib.request.urlopen("https://127.0.0.1:15678/api/mcp", context=ctx, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-mcp.json 2>/dev/null && \
       docker exec phanthymotus-sim-p0-perception python3 -c '
import json, urllib.request
data = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}}).encode()
request = urllib.request.Request(
    "http://127.0.0.1:15720/mcp",
    data=data,
    headers={"Content-Type": "application/json"},
)
print(urllib.request.urlopen(request, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-perception.json 2>/dev/null; then
      ready=1
      break
    fi
    sleep 2
  done

  if [[ "$ready" -ne 1 ]]; then
    printf 'service readiness timeout after 90s\n' >&2
    compose ps >&2 || true
    docker logs --tail 120 phanthymotus-sim-p0-agent-core >&2 || true
    docker logs --tail 120 phanthymotus-sim-p0-perception >&2 || true
    return 1
  fi

  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
ctx = ssl._create_unverified_context()
print(urllib.request.urlopen("https://127.0.0.1:15678/js/detail-panel.js", context=ctx, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-detail-panel.js
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
ctx = ssl._create_unverified_context()
print(urllib.request.urlopen("https://127.0.0.1:15678/js/canvas.js", context=ctx, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-canvas.js
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
ctx = ssl._create_unverified_context()
print(urllib.request.urlopen("https://127.0.0.1:15678/js/deploy-panel.js", context=ctx, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-deploy-panel.js
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import ssl, urllib.request
ctx = ssl._create_unverified_context()
print(urllib.request.urlopen("https://127.0.0.1:15678/api/drivers", context=ctx, timeout=3).read().decode())
' >/tmp/phanthymotus-sim-p0-drivers.json

  docker exec phanthymotus-sim-p0-agent-core uv pip check --python .venv/bin/python
  docker exec phanthymotus-sim-p0-agent-core docker --version
  docker exec phanthymotus-sim-p0-agent-core docker compose version
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python \
    /work/tests/test_local_services.py -v
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import docker
client = docker.from_env()
names = [
    "phanthymotus-sim-p0-agent-core",
    "phanthymotus-sim-p0-perception",
]
for optional in ("phanthymotus-sim-p2-g1-driver", "phanthymotus-sim-p3-gazebo-nav"):
    try:
        client.containers.get(optional)
    except docker.errors.NotFound:
        continue
    names.append(optional)
for name in names:
    container = client.containers.get(name)
    assert container.attrs["Name"].lstrip("/") == name
print("Agent Core Docker socket management PASS containers=" + ",".join(names))
'
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import importlib.metadata as metadata
import lark_oapi as lark
import lark_oapi.ws.client
from lark_oapi.api.im.v1 import CreateMessageRequest
assert hasattr(lark.ws, "Client")
version = metadata.version("lark-oapi")
assert tuple(int(part) for part in version.split(".")[:2]) >= (1, 4), version
print("Feishu Channel SDK runtime PASS version=" + version)
'
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
from pathlib import Path
source = Path("/work/src/channel/adapters/feishu.py").read_text()
needle = "aiohttp.ClientSession(timeout=timeout, trust_env=True)"
assert source.count(needle) == 2, source.count(needle)
print("Feishu REST proxy environment support PASS")
'
  docker exec phanthymotus-sim-p0-agent-core .venv/bin/python -c '
import json
import ssl
import time
import urllib.request

ctx = ssl._create_unverified_context()
url = "https://127.0.0.1:15678/api/channel/list"
deadline = time.monotonic() + 30
while True:
    payload = json.load(urllib.request.urlopen(url, context=ctx, timeout=3))
    enabled = [
        item for item in payload.get("channels", [])
        if item.get("platform") == "feishu" and item.get("enabled")
    ]
    if not enabled:
        print("Feishu Channel runtime SKIP: no enabled channel configured")
        break
    disconnected = [item["id"] for item in enabled if item.get("status") != "connected"]
    if not disconnected:
        print("Feishu Channel connection PASS ids=" + ",".join(item["id"] for item in enabled))
        break
    if time.monotonic() >= deadline:
        raise SystemExit("enabled Feishu Channel did not connect: " + ",".join(disconnected))
    time.sleep(2)
'

  python3 - <<'PY'
import json
core = json.load(open('/tmp/phanthymotus-sim-p0-mcp.json'))
perception = json.load(open('/tmp/phanthymotus-sim-p0-perception.json'))
web = open('/tmp/phanthymotus-sim-p0-web.html').read().lower()
detail_panel = open('/tmp/phanthymotus-sim-p0-detail-panel.js').read()
canvas = open('/tmp/phanthymotus-sim-p0-canvas.js').read()
deploy_panel = open('/tmp/phanthymotus-sim-p0-deploy-panel.js').read()
assert '<html' in web, web[:200]
assert core['code'] == 200, core
assert any(x.get('name') == 'Perception Stack' and x.get('url') == 'http://perception:15720/mcp' for x in core['data']), core
assert perception['result']['serverInfo']['name'] == 'perception-bundle', perception
assert "showTopicDetail(topicPath, format, mcpId = '')" in detail_panel
assert "_renderer.mount(body, mcpId || 'detail')" in detail_panel
assert canvas.count("topics[0].format || '', mcpId") == 2
assert '本地仿真' in deploy_panel
print('Core WebUI/API + Perception MCP registration PASS')
PY

  python3 - "$LOCAL_SERVICES_FILE" <<'PY'
import json
import sys

manifest = json.load(open(sys.argv[1]))
response = json.load(open('/tmp/phanthymotus-sim-p0-drivers.json'))
assert response['code'] == 200, response
actual = {item['id']: item for item in response['data'] if item.get('local_managed')}
expected = {item['id']: item for item in manifest}
assert actual.keys() == expected.keys(), (actual.keys(), expected.keys())
assert {'core', 'perception'} <= actual.keys(), actual
for driver_id, entry in actual.items():
    assert entry['image'] == expected[driver_id]['image'], (driver_id, entry)
    assert entry['status'] in {'running', 'exited', 'stopped'}, (driver_id, entry)
    assert entry['local_managed'] is True, (driver_id, entry)
print('Agent Core local services API PASS ids=' + ','.join(sorted(actual)))
PY

  local llm_probe_code
  llm_probe_code="$(docker exec phanthymotus-sim-p0-agent-core sh -lc \
    'curl --noproxy "" -x "$HTTPS_PROXY" -sS -o /dev/null -w "%{http_code}" --connect-timeout 5 --max-time 15 "$1"' \
    -- "$LLM_PROBE_URL")"
  if [[ "$llm_probe_code" != "401" ]]; then
    printf 'runtime LLM proxy probe failed: url=%s expected_http=401 actual_http=%s\n' \
      "$LLM_PROBE_URL" "$llm_probe_code" >&2
    return 1
  fi
  log "runtime LLM proxy PASS url=${LLM_PROBE_URL} http=401_without_key"

  docker inspect phanthymotus-sim-p0-agent-core phanthymotus-sim-p0-perception \
    | python3 -c '
import json, sys
expected_proxy = sys.argv[1]
items = json.load(sys.stdin)
expected = {
  "phanthymotus-sim-p0-agent-core": (2_000_000_000, 2 * 1024**3),
  "phanthymotus-sim-p0-perception": (4_000_000_000, 8 * 1024**3),
}
for item in items:
    name = item["Name"].lstrip("/")
    host = item["HostConfig"]
    assert host["Privileged"] is False, name
    assert host["NetworkMode"] == "phanthymotus-sim-p0-net", (name, host["NetworkMode"])
    assert not host.get("Devices"), (name, host.get("Devices"))
    assert not host.get("DeviceRequests"), (name, host.get("DeviceRequests"))
    assert host["NanoCpus"] == expected[name][0], (name, host["NanoCpus"])
    assert host["Memory"] == expected[name][1], (name, host["Memory"])
    assert "ROS_DOMAIN_ID=83" in item["Config"]["Env"], item["Config"]["Env"]
    if name == "phanthymotus-sim-p0-agent-core":
        env = dict(value.split("=", 1) for value in item["Config"]["Env"] if "=" in value)
        assert env["HTTP_PROXY"] == expected_proxy, env["HTTP_PROXY"]
        assert env["HTTPS_PROXY"] == expected_proxy, env["HTTPS_PROXY"]
        assert "agent-core" in env["NO_PROXY"] and "sim-driver" in env["NO_PROXY"], env["NO_PROXY"]
        assert "gazebo-nav" in env["NO_PROXY"], env["NO_PROXY"]
        assert env["COMPOSE_DIR"] == "/opt/phanthy-motus", env
        assert env["LOCAL_SERVICES_MANIFEST"].endswith("/local-services.json"), env
        mounts = {mount["Destination"]: mount for mount in item["Mounts"]}
        assert mounts["/var/run/docker.sock"]["RW"] is True, mounts
        assert mounts["/opt/phanthy-motus"]["RW"] is True, mounts
        assert mounts["/etc/phanthymotus/local-services"]["RW"] is False, mounts
    for bindings in (host.get("PortBindings") or {}).values():
        for binding in bindings:
            assert binding["HostIp"] == "127.0.0.1", (name, binding)
print("runtime topology + production-equivalent Core Docker authority PASS")
' "$RUNTIME_PROXY"

  compose ps
  log 'VERIFY PASS'
}

deploy_core_and_verify() {
  local previous_image=''
  if docker inspect phanthymotus-sim-p0-agent-core >/dev/null 2>&1; then
    previous_image="$(docker inspect phanthymotus-sim-p0-agent-core -f '{{.Config.Image}}')"
  fi

  build_core
  ensure_local_services_file
  if compose up -d --no-deps --force-recreate agent-core && \
      render_local_services && verify; then
    log "CORE DEPLOY + VERIFY PASS image=${CORE_IMAGE}"
    return 0
  else
    local rc=$?
    if [[ -n "$previous_image" ]]; then
      log "restoring previous Agent Core image=${previous_image}"
      CORE_IMAGE_OVERRIDE="$previous_image" \
        compose up -d --no-deps --force-recreate agent-core || true
      render_local_services || true
      docker inspect phanthymotus-sim-p0-agent-core \
        -f 'rollback state={{.State.Status}} image={{.Config.Image}}' || true
    fi
    return "$rc"
  fi
}

down() {
  compose down
  render_local_services
  log 'DOWN PASS; named volumes preserved'
}

case "${1:-}" in
  lock-contract) lock_contract ;;
  preflight) preflight ;;
  build-core) build_core ;;
  build) build ;;
  up) up ;;
  verify) verify ;;
  deploy-core-and-verify) deploy_core_and_verify ;;
  down) down ;;
  *)
    printf 'usage: %s {lock-contract|preflight|build-core|build|up|verify|deploy-core-and-verify|down}\n' "$0" >&2
    exit 2
    ;;
esac
