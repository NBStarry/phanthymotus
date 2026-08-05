#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
nav2_container="phanthy-nav2-shadow"
perception_container="embodied-perception"
dry_run="${DRY_RUN:-0}"
map_name="${MAP_NAME:-g1-n3-acceptance}"
goal_distance="${GOAL_DISTANCE_M:-0.6}"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${dry_run}" != "0" && "${dry_run}" != "1" ]]; then
  echo "ERROR=DRY_RUN must be 0 or 1" >&2
  exit 2
fi
if [[ ! "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$ ]]; then
  echo "ERROR=MAP_NAME must be a plain identifier" >&2
  exit 2
fi
if [[ ! "${goal_distance}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR=GOAL_DISTANCE_M must be a positive decimal" >&2
  exit 2
fi
if [[ "${dry_run}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the N5 MCP shadow acceptance" >&2
  exit 2
fi

REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  --env MCP_STARTUP_TIMEOUT=5 \
  --env EXPECT_BRIDGE_SUBSCRIBER=1 \
  "${perception_container}" python3 /tests/mcp_probe.py

runtime_python='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /nav2_ws/install/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec python3 -
'
printf -v quoted_runtime_python '%q' "${runtime_python}"
printf -v quoted_map_name '%q' "${map_name}"
printf -v quoted_goal_distance '%q' "${goal_distance}"
printf -v quoted_dry_run '%q' "${dry_run}"

ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec -i --env N5_MAP_NAME=${quoted_map_name} --env N5_GOAL_DISTANCE_M=${quoted_goal_distance} --env N5_DRY_RUN=${quoted_dry_run} ${nav2_container} /bin/bash -lc ${quoted_runtime_python}" \
  <"${nav2_dir}/tests/n5_mcp_acceptance_probe.py"

REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
if [[ "${dry_run}" == "1" ]]; then
  echo "GENERAL_NAVIGATION_N5_MCP_ACCEPTANCE_PREFLIGHT=PASS"
  echo "NOTE=read-only costmap goal selection passed; no navigation command was published"
  exit 0
fi
echo "GENERAL_NAVIGATION_N5_G1_SHADOW_ACCEPTANCE=PASS"
echo "NOTE=MCP goal was stopped; no Driver executor was connected"
