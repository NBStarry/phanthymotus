#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
preflight_only="${PREFLIGHT_ONLY:-0}"
container_name="phanthy-motus-agent-core-1"
compose_file="/opt/phanthy-motus/docker-compose.yml"
backup_file="${compose_file}.before-navigation-core-$(date +%Y%m%dT%H%M%S)"

set -a
. "${deploy_dir}/source-lock.env"
set +a

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

remote_container_inspect() {
  local format="$1"
  local container="$2"
  local quoted_format
  local quoted_container
  printf -v quoted_format '%q' "${format}"
  printf -v quoted_container '%q' "${container}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker container inspect --format ${quoted_format} ${quoted_container}"
}

if [[ "${preflight_only}" != "0" && "${preflight_only}" != "1" ]]; then
  echo "ERROR=PREFLIGHT_ONLY must be 0 or 1" >&2
  exit 2
fi
if [[ "${preflight_only}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the Agent Core upgrade" >&2
  exit 2
fi

image_meta="$(docker image inspect "${AGENT_CORE_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${AGENT_CORE_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -f "${compose_file}"; then
  echo "ERROR=${compose_file} is absent" >&2
  exit 1
fi
compose_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet

current_meta="$(remote_container_inspect \
  '{{.State.Running}}|{{.Config.Image}}' "${container_name}")"
IFS='|' read -r current_running current_image <<<"${current_meta}"
current_image_id="$(remote_container_inspect '{{.Image}}' "${container_name}")"
if [[ "${current_running}" != "true" ]]; then
  echo "ERROR=${container_name} must be running before upgrade" >&2
  exit 1
fi

compose_probe="import yaml; d=yaml.safe_load(open('${compose_file}')) or {}; s=d.get('services',{}); assert 'agent-core' in s; print(s['agent-core'].get('image',''))"
printf -v quoted_compose_probe '%q' "${compose_probe}"
compose_image="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} python3 -c ${quoted_compose_probe}")"
if [[ "${compose_image}" != "${current_image}" ]]; then
  echo "ERROR=compose/runtime Core image mismatch: ${compose_image} != ${current_image}" >&2
  exit 1
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${container_name}" python3 - \
  <"${deploy_dir}/tests/project_stopped_probe.py"

docker_root="$(ssh "${ssh_opts[@]}" "${g1_host}" docker info \
  --format '{{.DockerRootDir}}')"
remote_free_kib="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "df -Pk ${docker_root} | tail -n 1 | awk '{print \$4}'")"
if [[ ! "${remote_free_kib}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=could not determine remote Docker free space" >&2
  exit 1
fi
required_kib=$((image_size / 1024 + 256 * 1024))
if ((remote_free_kib < required_kib)); then
  echo "ERROR=remote Docker has ${remote_free_kib} KiB; need ${required_kib} KiB" >&2
  exit 1
fi

echo "AGENT_CORE_CURRENT_IMAGE=${current_image}"
echo "AGENT_CORE_TARGET_IMAGE=${AGENT_CORE_IMAGE}"
echo "AGENT_CORE_TARGET_ID=${image_id}"
echo "AGENT_CORE_TARGET_SIZE=${image_size}"
echo "AGENT_CORE_REMOTE_FREE_KIB=${remote_free_kib}"
echo "AGENT_CORE_COMPOSE_OWNERSHIP=${compose_stat}"

if [[ "${current_image}" == "${AGENT_CORE_IMAGE}" && \
      "${current_image_id}" == "${image_id}" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
    --env "EXPECTED_IMAGE_TAG=${AGENT_CORE_IMAGE_TAG}" \
    "${container_name}" python3 - \
    <"${deploy_dir}/tests/runtime_probe.py"
  echo "AGENT_CORE_NAVIGATION_UPGRADE=ALREADY_CURRENT"
  echo "NOTE=trusted navigation orchestration is already live; no state changed"
  exit 0
fi
if [[ "${preflight_only}" == "1" ]]; then
  echo "AGENT_CORE_NAVIGATION_UPGRADE_PREFLIGHT=PASS"
  echo "NOTE=read-only; no image, compose, container, project, or robot state changed"
  exit 0
fi

"${script_dir}/smoke-test.sh"
echo "[agent-core-navigation] loading ${AGENT_CORE_IMAGE} before Core replacement"
docker save "${AGENT_CORE_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_arch="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${AGENT_CORE_IMAGE}" --format '{{.Architecture}}')"
if [[ "${remote_arch}" != "arm64" ]]; then
  echo "ERROR=remote ${AGENT_CORE_IMAGE} architecture is ${remote_arch}" >&2
  exit 1
fi

rollback_needed=0
restore_program="import os,shutil; p='${compose_file}'; b='${backup_file}'; st=os.stat(p); t=p+'.navigation-core-rollback.tmp'; shutil.copyfile(b,t); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
printf -v quoted_restore_program '%q' "${restore_program}"
printf -v quoted_agent_core_image '%q' "${AGENT_CORE_IMAGE}"
rollback() {
  rc=$?
  trap - ERR
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[agent-core-navigation] upgrade failed; restoring compose backup" >&2
    restore_command="docker run --rm --network none --read-only --mount type=bind,source=/opt/phanthy-motus,target=/opt/phanthy-motus --entrypoint python3 ${quoted_agent_core_image} -c ${quoted_restore_program}"
    if ! {
      ssh "${ssh_opts[@]}" "${g1_host}" "${restore_command}"
      ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
        --file "${compose_file}" config --quiet
      ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
        --file "${compose_file}" up --detach --no-deps agent-core
    }; then
      echo "ERROR=automatic Core rollback failed; backup retained at ${backup_file}" >&2
    fi
  fi
  exit "${rc}"
}
trap rollback ERR

ssh "${ssh_opts[@]}" "${g1_host}" cp -p "${compose_file}" "${backup_file}"
rollback_needed=1

update_program="import os,sys,yaml; p='${compose_file}'; st=os.stat(p); d=yaml.safe_load(open(p)) or {}; s=d.get('services',{}); assert 'agent-core' in s; assert s['agent-core'].get('image')==sys.argv[1]; s['agent-core']['image']=sys.argv[2]; t=p+'.navigation-core.tmp'; h=open(t,'w'); yaml.safe_dump(d,h,default_flow_style=False,allow_unicode=True,sort_keys=False); h.close(); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
printf -v quoted_update_program '%q' "${update_program}"
printf -v quoted_current_image '%q' "${current_image}"
printf -v quoted_target_image '%q' "${AGENT_CORE_IMAGE}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} python3 -c ${quoted_update_program} ${quoted_current_image} ${quoted_target_image}"

updated_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
if [[ "${updated_stat}" != "${compose_stat}" ]]; then
  echo "ERROR=compose ownership changed: ${compose_stat} -> ${updated_stat}" >&2
  false
fi
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" up --detach --no-deps agent-core

for _ in $(seq 1 40); do
  running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    --format '{{.State.Running}}' "${container_name}" 2>/dev/null || true)"
  if [[ "${running}" == "true" ]]; then
    break
  fi
  sleep 1
done
if [[ "${running:-false}" != "true" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker logs --tail 240 \
    "${container_name}" >&2 || true
  echo "ERROR=${container_name} did not start" >&2
  false
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  --env "EXPECTED_IMAGE_TAG=${AGENT_CORE_IMAGE_TAG}" \
  "${container_name}" python3 - \
  <"${deploy_dir}/tests/runtime_probe.py"

rollback_needed=0
trap - ERR
echo "AGENT_CORE_NAVIGATION_UPGRADE=PASS"
echo "AGENT_CORE_COMPOSE_BACKUP=${backup_file}"
echo "NOTE=trusted navigation lifecycle is live and the canvas project remains stopped; no robot command was issued"
