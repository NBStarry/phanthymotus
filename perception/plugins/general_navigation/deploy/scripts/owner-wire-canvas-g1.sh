#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
stage="${STAGE:-preflight}"
core_container="phanthy-motus-agent-core-1"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${stage}" != "preflight" && "${stage}" != "wire" ]]; then
  echo "ERROR=STAGE must be preflight or wire" >&2
  exit 2
fi
if [[ "${stage}" == "wire" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=wire requires I_AM_G1_OWNER=1" >&2
  exit 2
fi

running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}}' "${core_container}" 2>/dev/null || true)"
if [[ "${running}" != "true" ]]; then
  echo "ERROR=${core_container} is not running" >&2
  exit 1
fi

apply=0
if [[ "${stage}" == "wire" ]]; then
  apply=1
fi
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  --env "CANVAS_APPLY=${apply}" \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/canvas_wire.py"

if [[ "${stage}" == "wire" ]]; then
  echo "NEXT=manually start the canvas project, then run owner-loco-card-acceptance.sh with STAGE=preflight"
  echo "NOTE=the four navigation cards and three required links were refreshed; an existing compatible goal_pose link is preserved; the project remains stopped"
fi
