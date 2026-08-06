#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
nav2_container="phanthy-nav2-shadow"
stage="${STAGE:-preflight}"
map_name="${MAP_NAME:-g1-n3-acceptance}"
goal_distance="${GOAL_DISTANCE_M:-0.6}"
loco_proposal_node="${LOCO_PROPOSAL_NODE:-g1_loco_velocity_proposal}"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${stage}" != "preflight" && "${stage}" != "move" ]]; then
  echo "ERROR=STAGE must be preflight or move" >&2
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
if [[ ! "${loco_proposal_node}" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "ERROR=LOCO_PROPOSAL_NODE must be a ROS node basename" >&2
  exit 2
fi
if [[ "${stage}" == "move" && \
      ( "${I_AM_G1_OWNER:-0}" != "1" || \
        "${I_HAVE_G1_REMOTE:-0}" != "1" ) ]]; then
  echo "ERROR=move requires I_AM_G1_OWNER=1 and I_HAVE_G1_REMOTE=1" >&2
  exit 2
fi

G1_HOST="${g1_host}" \
LOCO_PROPOSAL_NODE="${loco_proposal_node}" \
  "${script_dir}/loco-integration-readiness.sh"

agent_core_access_token="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "sed -n 's/^ACCESS_TOKEN=//p' /opt/phanthy-motus/.env | head -n 1")"
if [[ -z "${agent_core_access_token}" ]]; then
  echo "ERROR=Agent Core ACCESS_TOKEN is unavailable" >&2
  exit 1
fi

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
printf -v quoted_access_token '%q' "${agent_core_access_token}"

dry_run=1
physical_e2e=0
if [[ "${stage}" == "move" ]]; then
  dry_run=0
  physical_e2e=1
fi

ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec -i --env N5_COMMAND_BACKEND=agent_core --env AGENT_CORE_ACCESS_TOKEN=${quoted_access_token} --env N5_MAP_NAME=${quoted_map_name} --env N5_GOAL_DISTANCE_M=${quoted_goal_distance} --env N5_DRY_RUN=${dry_run} --env N5_PHYSICAL_E2E=${physical_e2e} ${nav2_container} /bin/bash -lc ${quoted_runtime_python}" \
  <"${nav2_dir}/tests/n5_mcp_acceptance_probe.py"

if [[ "${stage}" == "preflight" ]]; then
  echo "GENERAL_NAVIGATION_LOCO_E2E_PREFLIGHT=PASS"
  echo "NEXT=activate G1 main control, keep the remote in hand, then rerun with STAGE=move"
  echo "NOTE=read-only; the live costmap goal was selected but no navigation command was issued"
  exit 0
fi

G1_HOST="${g1_host}" \
LOCO_PROPOSAL_NODE="${loco_proposal_node}" \
  "${script_dir}/loco-integration-readiness.sh"
echo "GENERAL_NAVIGATION_LOCO_CARD_ACCEPTANCE=PASS"
echo "NOTE=the goal was issued through Agent Core -> navigation2 -> Driver loco; measured motion, arrival, Driver stop confirmation, and terminal zero proposal all passed"
