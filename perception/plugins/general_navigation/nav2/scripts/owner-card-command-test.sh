#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
dry_run="${DRY_RUN:-0}"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${dry_run}" != "0" && "${dry_run}" != "1" ]]; then
  echo "ERROR=DRY_RUN must be 0 or 1" >&2
  exit 2
fi
if [[ "${dry_run}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the G1 card command test" >&2
  exit 2
fi

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

if [[ "${dry_run}" == "1" ]]; then
  echo "G1_NAV2_CARD_COMMAND_PREFLIGHT=PASS"
  echo "NOTE=read-only; no navigation command was published"
  exit 0
fi

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

ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec -i ${container_name} /bin/bash -lc ${quoted_runtime_python}" \
  <"${nav2_dir}/tests/navigation_command_probe.py"

echo "G1_NAV2_G1_CARD_COMMAND_TEST=PASS"
echo "NOTE=JSON bridge was exercised in shadow; no Driver executor was connected"
