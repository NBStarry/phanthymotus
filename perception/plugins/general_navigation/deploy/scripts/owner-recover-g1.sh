#!/usr/bin/env bash
set -Eeuo pipefail

g1_host="${G1_HOST:-g1-sh-wifi}"
preflight_only="${PREFLIGHT_ONLY:-0}"
core_container="phanthy-motus-agent-core-1"
perception_container="embodied-perception"
compose_file="/opt/phanthy-motus/docker-compose.yml"
backup_file="${RECOVERY_BACKUP:-}"
expected_current_sha="${RECOVERY_CURRENT_SHA256:-}"
expected_backup_sha="${RECOVERY_BACKUP_SHA256:-}"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${preflight_only}" != "0" && "${preflight_only}" != "1" ]]; then
  echo "ERROR=PREFLIGHT_ONLY must be 0 or 1" >&2
  exit 2
fi
if [[ "${preflight_only}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize G1 compose recovery" >&2
  exit 2
fi
case "${backup_file}" in
  /opt/phanthy-motus/docker-compose.yml.before-general-navigation-*) ;;
  *)
    echo "ERROR=RECOVERY_BACKUP must name one general-navigation compose backup" >&2
    exit 2
    ;;
esac
if [[ -z "${expected_current_sha}" || -z "${expected_backup_sha}" ]]; then
  echo "ERROR=RECOVERY_CURRENT_SHA256 and RECOVERY_BACKUP_SHA256 are required" >&2
  exit 2
fi

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -f "${compose_file}"; then
  echo "ERROR=${compose_file} is absent" >&2
  exit 1
fi
if ! ssh "${ssh_opts[@]}" "${g1_host}" test -f "${backup_file}"; then
  echo "ERROR=${backup_file} is absent" >&2
  exit 1
fi

read -r current_sha _ <<<"$(ssh "${ssh_opts[@]}" "${g1_host}" \
  sha256sum "${compose_file}")"
read -r backup_sha _ <<<"$(ssh "${ssh_opts[@]}" "${g1_host}" \
  sha256sum "${backup_file}")"
if [[ "${backup_sha}" != "${expected_backup_sha}" ]]; then
  echo "ERROR=backup hash changed: ${backup_sha}" >&2
  exit 1
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${backup_file}" config --quiet

if ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    "${perception_container}" >/dev/null 2>&1; then
  echo "ERROR=${perception_container} exists; refusing compose-only recovery" >&2
  exit 1
fi
port_listeners="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "ss -ltnH 'sport = :15720'")"
if [[ -n "${port_listeners}" ]]; then
  printf '%s\n' "${port_listeners}" >&2
  echo "ERROR=TCP port 15720 is listening; refusing compose-only recovery" >&2
  exit 1
fi

if [[ "${current_sha}" == "${backup_sha}" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
    --file "${compose_file}" config --quiet
  echo "GENERAL_NAVIGATION_RECOVERY=ALREADY_RESTORED"
  exit 0
fi
if [[ "${current_sha}" != "${expected_current_sha}" ]]; then
  echo "ERROR=current compose hash changed unexpectedly: ${current_sha}" >&2
  exit 1
fi

state_probe="import json,yaml; c=yaml.safe_load(open('${compose_file}')) or {}; b=yaml.safe_load(open('${backup_file}')) or {}; cs=c.get('services',{}); bs=b.get('services',{}); assert set(cs)=={'agent-core','perception'}, set(cs); assert set(bs)=={'agent-core'}, set(bs); assert cs['perception'].get('image')=='phanthy-perception:g1-general-navigation1'; print(json.dumps({'current_services':sorted(cs),'backup_services':sorted(bs)}))"
printf -v quoted_state_probe '%q' "${state_probe}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_state_probe}"

current_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
backup_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${backup_file}")"
echo "GENERAL_NAVIGATION_RECOVERY_CURRENT_SHA256=${current_sha}"
echo "GENERAL_NAVIGATION_RECOVERY_BACKUP_SHA256=${backup_sha}"
echo "GENERAL_NAVIGATION_RECOVERY_OWNERSHIP=${current_stat}->${backup_stat}"

if [[ "${preflight_only}" == "1" ]]; then
  echo "GENERAL_NAVIGATION_RECOVERY_PREFLIGHT=PASS"
  echo "NOTE=read-only; exact backup is valid and no Perception container exists"
  exit 0
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker exec "${core_container}" \
  cp -p "${backup_file}" "${compose_file}"

ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet
read -r restored_sha _ <<<"$(ssh "${ssh_opts[@]}" "${g1_host}" \
  sha256sum "${compose_file}")"
restored_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
if [[ "${restored_sha}" != "${backup_sha}" || \
      "${restored_stat}" != "${backup_stat}" ]]; then
  echo "ERROR=recovery verification failed: sha=${restored_sha}, stat=${restored_stat}" >&2
  exit 1
fi

echo "GENERAL_NAVIGATION_RECOVERY=PASS"
echo "GENERAL_NAVIGATION_RECOVERY_RESTORED=${backup_file}"
echo "NOTE=compose restored; no container was started or robot command published"
