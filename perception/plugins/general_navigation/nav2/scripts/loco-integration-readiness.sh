#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
deploy_dir="$(cd "${nav2_dir}/../deploy" && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
loco_proposal_node="${LOCO_PROPOSAL_NODE:-g1_loco_velocity_proposal}"
perception_container="embodied-perception"
core_container="phanthy-motus-agent-core-1"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ ! "${loco_proposal_node}" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "ERROR=LOCO_PROPOSAL_NODE must be a ROS node basename" >&2
  exit 2
fi
REQUIRE_DRIVER_INPUT_CONTRACT=1 \
G1_HOST="${g1_host}" "${script_dir}/g1-readiness.sh"
REQUIRE_N5_PROTOCOL=1 \
PROPOSAL_DRIVER_NODE="${loco_proposal_node}" \
PROPOSAL_DRIVER_STANDBY=1 \
G1_HOST="${g1_host}" \
  "${script_dir}/audit-shadow.sh"

ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  --env MCP_STARTUP_TIMEOUT=5 \
  --env EXPECT_BRIDGE_SUBSCRIBER=1 \
  --env EXPECT_CANVAS_WIRED=1 \
  "${perception_container}" python3 - \
  <"${deploy_dir}/tests/mcp_probe.py"

ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/core_registry_probe.py"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/loco_registry_probe.py"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/loco_runtime_probe.py"

echo "GENERAL_NAVIGATION_LOCO_NODE=${loco_proposal_node}"
echo "GENERAL_NAVIGATION_LOCO_LINK_PREFLIGHT=PASS"
echo "NOTE=read-only; legacy sensor inputs, canvas wiring, and the released Driver loco standby contract passed; no navigation command was issued"
