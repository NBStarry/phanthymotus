#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  CORE_TOKEN=... ./deploy/deploy_kws_interrupt.sh \
    <ssh-target> <g1-image> <perception-image> <core-image>

Example:
  CORE_TOKEN=... ./deploy/deploy_kws_interrupt.sh \
    g1-shanghai \
    registry/namespace/drivers/unitree/g1:release.YYMMDD.SHA \
    registry/namespace/perception:release.YYMMDD.SHA-jetson-jp5.11 \
    registry/namespace/core:release.YYMMDD.SHA

CORE_TOKEN is optional when Agent Core authentication is disabled.
LOCAL_PORT defaults to 25678.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ $# -ne 4 ]]; then
    usage >&2
    exit 2
fi

SSH_TARGET="$1"
G1_IMAGE="$2"
PERCEPTION_IMAGE="$3"
CORE_IMAGE="$4"
LOCAL_PORT="${LOCAL_PORT:-25678}"

[[ "${LOCAL_PORT}" =~ ^[0-9]+$ ]] || {
    echo "LOCAL_PORT must be a number" >&2
    exit 2
}

for command in ssh curl python3; do
    command -v "${command}" >/dev/null || {
        echo "Missing command: ${command}" >&2
        exit 1
    }
done

AUTH_ARGS=()
if [[ -n "${CORE_TOKEN:-}" ]]; then
    AUTH_ARGS=(-H "Authorization: Bearer ${CORE_TOKEN}")
fi

BASE_URL="http://127.0.0.1:${LOCAL_PORT}"
TUNNEL_PID=""
cleanup() {
    [[ -z "${TUNNEL_PID}" ]] || kill "${TUNNEL_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ssh -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
    -N -L "${LOCAL_PORT}:127.0.0.1:15678" "${SSH_TARGET}" &
TUNNEL_PID=$!

ready=false
for _ in {1..20}; do
    if curl -fsS --max-time 2 "${AUTH_ARGS[@]}" \
        "${BASE_URL}/api/auth/verify" >/dev/null; then
        ready=true
        break
    fi
    kill -0 "${TUNNEL_PID}" 2>/dev/null || break
    sleep 0.5
done
[[ "${ready}" == true ]] || {
    echo "Agent Core is unreachable or CORE_TOKEN is invalid" >&2
    exit 1
}

post_image() {
    local path="$1" image="$2" response
    echo "Deploying ${image}"
    response=$(curl -fsS --max-time 900 "${AUTH_ARGS[@]}" \
        -H 'Content-Type: application/json' \
        -X POST "${BASE_URL}${path}" \
        --data "$(python3 -c 'import json,sys; print(json.dumps({"image": sys.argv[1]}))' "${image}")")
    python3 -c 'import json,sys; d=json.load(sys.stdin); assert d.get("code") == 200, d' \
        <<<"${response}"
}

check_running() {
    local driver_id="$1" expected="$2" response
    response=$(curl -fsS --max-time 10 "${AUTH_ARGS[@]}" \
        "${BASE_URL}/api/drivers/${driver_id}/status")
    python3 -c '
import json, sys
d = json.load(sys.stdin)
status = d.get("data", {})
assert d.get("code") == 200 and status.get("status") == "running", d
actual = status.get("running_image", "")
assert not actual or actual == sys.argv[1], {"expected": sys.argv[1], "actual": actual}
' "${expected}" <<<"${response}"
}

post_image '/api/drivers/g1-driver/deploy' "${G1_IMAGE}"
check_running 'g1-driver' "${G1_IMAGE}"

post_image '/api/drivers/perception/deploy' "${PERCEPTION_IMAGE}"
check_running 'perception' "${PERCEPTION_IMAGE}"

post_image '/api/system/update' "${CORE_IMAGE}"
echo "Core update started; waiting for the new image..."
for _ in {1..90}; do
    sleep 2
    response=$(curl -fsS --max-time 5 "${AUTH_ARGS[@]}" \
        "${BASE_URL}/api/drivers" 2>/dev/null || true)
    if python3 -c '
import json, sys
try:
    items = json.load(sys.stdin).get("data", [])
except Exception:
    raise SystemExit(1)
raise SystemExit(0 if any(x.get("category") == "core" and x.get("running_image") == sys.argv[1] for x in items) else 1)
' "${CORE_IMAGE}" <<<"${response}"; then
        echo "Deployment complete: ${SSH_TARGET}"
        exit 0
    fi
done

echo "Core did not report the expected image within 180 seconds" >&2
exit 1
