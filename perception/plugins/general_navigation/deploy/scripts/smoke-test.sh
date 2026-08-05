#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
container_name="general-navigation-perception-smoke-$$"

set -a
. "${deploy_dir}/source-lock.env"
set +a

cleanup() {
  docker rm --force "${container_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

image_arch="$(docker image inspect "${GENERAL_NAVIGATION_IMAGE}" \
  --format '{{.Architecture}}')"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${GENERAL_NAVIGATION_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

docker run --detach \
  --name "${container_name}" \
  --network none \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=64m \
  --tmpfs /root/.ros:rw,nosuid,nodev,noexec,size=32m \
  --env AGENT_CORE_URL=https://127.0.0.1:9 \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
  --env "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}" \
  --cap-drop ALL \
  --security-opt no-new-privileges:true \
  --pids-limit 128 \
  --memory 512m \
  --cpus 1 \
  "${GENERAL_NAVIGATION_IMAGE}" >/dev/null

probe_output=""
for _ in $(seq 1 30); do
  if probe_output="$(docker exec "${container_name}" \
      python3 /tests/mcp_probe.py 2>&1)"; then
    break
  fi
  sleep 1
done
if [[ "${probe_output}" != *"GENERAL_NAVIGATION_MCP_PROBE=PASS"* ]]; then
  printf '%s\n' "${probe_output}" >&2
  docker logs "${container_name}" >&2
  echo "ERROR=navigation-only MCP probe did not pass" >&2
  exit 1
fi

logs="$(docker logs "${container_name}" 2>&1)"
if [[ "${logs}" != *"GeneralNavigationPlugin loaded (namespace=ubuntu)"* ]]; then
  printf '%s\n' "${logs}" >&2
  echo "ERROR=general_navigation plugin did not load" >&2
  exit 1
fi
if [[ "${logs}" != *"WebSocket ASR server disabled by config"* ]]; then
  printf '%s\n' "${logs}" >&2
  echo "ERROR=navigation-only image did not disable WebSocket ASR" >&2
  exit 1
fi
if [[ "${logs}" == *"WebSocket ASR server →"* ]]; then
  printf '%s\n' "${logs}" >&2
  echo "ERROR=WebSocket ASR unexpectedly started" >&2
  exit 1
fi

printf '%s\n' "${probe_output}"
echo "GENERAL_NAVIGATION_CONTAINER_SMOKE=PASS"
