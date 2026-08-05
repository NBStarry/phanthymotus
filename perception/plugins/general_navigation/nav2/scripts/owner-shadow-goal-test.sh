#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
dry_run="${DRY_RUN:-0}"
goal_distance="${GOAL_DISTANCE_M:-0.6}"
observe_sec="${GOAL_OBSERVE_SEC:-8.0}"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${dry_run}" != "0" && "${dry_run}" != "1" ]]; then
  echo "ERROR=DRY_RUN must be 0 or 1" >&2
  exit 2
fi
if [[ ! "${goal_distance}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR=GOAL_DISTANCE_M must be a positive decimal" >&2
  exit 2
fi
if [[ ! "${observe_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR=GOAL_OBSERVE_SEC must be a positive decimal" >&2
  exit 2
fi
if [[ "${dry_run}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the isolated Nav2 shadow goal" >&2
  exit 2
fi

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

runtime_python='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec python3 -
'
printf -v quoted_runtime_python '%q' "${runtime_python}"
printf -v quoted_dry_run '%q' "${dry_run}"
printf -v quoted_goal_distance '%q' "${goal_distance}"
printf -v quoted_observe_sec '%q' "${observe_sec}"

ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec -i --env G1_NAV2_GOAL_DRY_RUN=${quoted_dry_run} --env G1_NAV2_GOAL_DISTANCE_M=${quoted_goal_distance} --env G1_NAV2_GOAL_OBSERVE_SEC=${quoted_observe_sec} ${container_name} /bin/bash -lc ${quoted_runtime_python}" \
  <"${script_dir}/shadow_goal_test.py"
