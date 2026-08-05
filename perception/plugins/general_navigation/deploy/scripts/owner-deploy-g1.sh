#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
nav2_scripts="$(cd "${deploy_dir}/../nav2/scripts" && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
preflight_only="${PREFLIGHT_ONLY:-0}"
core_container="phanthy-motus-agent-core-1"
nav2_container="phanthy-nav2-shadow"
perception_container="embodied-perception"
compose_file="/opt/phanthy-motus/docker-compose.yml"
candidate_fragment="/opt/phanthy-motus/.general-navigation-service.candidate.yml"
backup_file="${compose_file}.before-general-navigation-$(date +%Y%m%dT%H%M%S)"

set -a
. "${deploy_dir}/source-lock.env"
set +a

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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the G1 Perception deployment" >&2
  exit 2
fi

image_meta="$(docker image inspect "${GENERAL_NAVIGATION_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${GENERAL_NAVIGATION_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

echo "[general-navigation] target=${g1_host}"
ssh "${ssh_opts[@]}" "${g1_host}" hostnamectl
ssh "${ssh_opts[@]}" "${g1_host}" docker ps \
  --format '{{.Names}},{{.Image}},{{.Status}}'

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -f "${compose_file}"; then
  echo "ERROR=${compose_file} is absent" >&2
  exit 1
fi
compose_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
IFS=':' read -r compose_uid compose_gid compose_mode <<<"${compose_stat}"
if [[ ! "${compose_uid}" =~ ^[0-9]+$ || \
      ! "${compose_gid}" =~ ^[0-9]+$ || \
      ! "${compose_mode}" =~ ^[0-7]+$ ]]; then
  echo "ERROR=could not parse compose ownership: ${compose_stat}" >&2
  exit 1
fi
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet

nav2_running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}}' "${nav2_container}")"
if [[ "${nav2_running}" != "true" ]]; then
  echo "ERROR=${nav2_container} is not running" >&2
  exit 1
fi

G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"

if ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    "${perception_container}" >/dev/null 2>&1; then
  echo "ERROR=${perception_container} already exists; refusing first-deploy overwrite" >&2
  exit 1
fi

compose_probe="import yaml; d=yaml.safe_load(open('${compose_file}')) or {}; print('present' if 'perception' in d.get('services', {}) else 'absent')"
printf -v quoted_compose_probe '%q' "${compose_probe}"
compose_service="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_compose_probe}")"
if [[ "${compose_service}" != "absent" ]]; then
  echo "ERROR=compose already contains perception; refusing first-deploy overwrite" >&2
  exit 1
fi

port_listeners="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "ss -ltnH 'sport = :15720'")"
if [[ -n "${port_listeners}" ]]; then
  printf '%s\n' "${port_listeners}" >&2
  echo "ERROR=TCP port 15720 is already listening" >&2
  exit 1
fi

docker_root="$(ssh "${ssh_opts[@]}" "${g1_host}" docker info \
  --format '{{.DockerRootDir}}')"
docker_disk_line="$(ssh "${ssh_opts[@]}" "${g1_host}" df -Pk \
  "${docker_root}" | tail -n 1)"
read -r _ _ _ docker_available_kb _ <<<"${docker_disk_line}"
if [[ ! "${docker_available_kb}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=could not parse Docker filesystem availability: ${docker_disk_line}" >&2
  exit 1
fi
minimum_available_kb=$((image_size / 1024 + 512 * 1024))
if (( docker_available_kb < minimum_available_kb )); then
  echo "ERROR=Docker filesystem has ${docker_available_kb} KiB free; need at least ${minimum_available_kb} KiB" >&2
  exit 1
fi

echo "GENERAL_NAVIGATION_IMAGE=${GENERAL_NAVIGATION_IMAGE}"
echo "GENERAL_NAVIGATION_IMAGE_ID=${image_id}"
echo "GENERAL_NAVIGATION_IMAGE_SIZE=${image_size}"
echo "GENERAL_NAVIGATION_REMOTE_DOCKER_FREE_KIB=${docker_available_kb}"
echo "GENERAL_NAVIGATION_REMOTE_PORT_15720=free"
echo "GENERAL_NAVIGATION_REMOTE_COMPOSE_SERVICE=absent"

if [[ "${preflight_only}" == "1" ]]; then
  echo "GENERAL_NAVIGATION_G1_PREFLIGHT=PASS"
  echo "NOTE=read-only preflight; no G1 file, image, compose, or container was changed"
  exit 0
fi

"${script_dir}/smoke-test.sh"

echo "[general-navigation] loading ${GENERAL_NAVIGATION_IMAGE} before compose changes"
docker save "${GENERAL_NAVIGATION_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_image_arch="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${GENERAL_NAVIGATION_IMAGE}" --format '{{.Architecture}}')"
if [[ "${remote_image_arch}" != "arm64" ]]; then
  echo "ERROR=remote image architecture is ${remote_image_arch}, expected arm64" >&2
  exit 1
fi

rollback_needed=0
fragment_container="general-navigation-fragment-$$"
rollback() {
  rc=$?
  trap - ERR
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force \
    "${fragment_container}" >/dev/null 2>&1 || true
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[general-navigation] deployment failed; restoring compose backup" >&2
    rollback_script="set -e; docker rm --force ${perception_container} >/dev/null 2>&1 || true; docker exec ${core_container} cp -p ${backup_file} ${compose_file}; docker compose --file ${compose_file} config --quiet"
    ssh "${ssh_opts[@]}" "${g1_host}" "${rollback_script}" || \
      echo "ERROR=automatic rollback failed; backup retained at ${backup_file}" >&2
  fi
  exit "${rc}"
}
trap rollback ERR

ssh "${ssh_opts[@]}" "${g1_host}" docker exec "${core_container}" \
  cp -p "${compose_file}" "${backup_file}"
rollback_needed=1

ssh "${ssh_opts[@]}" "${g1_host}" docker create \
  --name "${fragment_container}" "${GENERAL_NAVIGATION_IMAGE}" >/dev/null
ssh "${ssh_opts[@]}" "${g1_host}" docker cp \
  "${fragment_container}:/deploy/service.yml" "${candidate_fragment}"
ssh "${ssh_opts[@]}" "${g1_host}" docker rm "${fragment_container}" >/dev/null

merge_program="import os,sys,yaml; p='${compose_file}'; f='${candidate_fragment}'; st=os.stat(p); d=yaml.safe_load(open(p)) or {}; s=yaml.safe_load(open(f)) or {}; assert set(s)=={'perception'}; s['perception']['image']=sys.argv[1]; s['perception']['read_only']=False; d.setdefault('services',{}).update(s); t=p+'.general-navigation.tmp'; h=open(t,'w'); yaml.safe_dump(d,h,default_flow_style=False,allow_unicode=True,sort_keys=False); h.close(); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
printf -v quoted_merge_program '%q' "${merge_program}"
printf -v quoted_image '%q' "${GENERAL_NAVIGATION_IMAGE}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_merge_program} ${quoted_image}"

merged_compose_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
if [[ "${merged_compose_stat}" != "${compose_stat}" ]]; then
  echo "ERROR=compose ownership changed: ${compose_stat} -> ${merged_compose_stat}" >&2
  false
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" up --detach --no-deps perception

container_running="false"
for _ in $(seq 1 30); do
  container_running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    --format '{{.State.Running}}' "${perception_container}" 2>/dev/null || true)"
  if [[ "${container_running}" == "true" ]]; then
    break
  fi
  sleep 1
done
if [[ "${container_running}" != "true" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker logs --tail 200 \
    "${perception_container}" >&2 || true
  echo "ERROR=${perception_container} did not reach running state" >&2
  false
fi

runtime_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.Config.Image}},{{.HostConfig.ReadonlyRootfs}},{{.HostConfig.Privileged}},{{.HostConfig.PidMode}},{{.HostConfig.RestartPolicy.Name}}' \
  "${perception_container}")"
IFS=',' read -r runtime_image runtime_read_only runtime_privileged \
  runtime_pid_mode runtime_restart <<<"${runtime_meta}"
if [[ "${runtime_image}" != "${GENERAL_NAVIGATION_IMAGE}" || \
      "${runtime_read_only}" != "false" || \
      "${runtime_privileged}" != "false" || \
      -n "${runtime_pid_mode}" || \
      "${runtime_restart}" != "no" ]]; then
  echo "ERROR=unsafe or unexpected Perception runtime: ${runtime_meta}" >&2
  false
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  --env MCP_STARTUP_TIMEOUT=30 \
  --env EXPECT_BRIDGE_SUBSCRIBER=1 \
  "${perception_container}" python3 /tests/mcp_probe.py

registration_ready=0
for _ in $(seq 1 20); do
  if ssh "${ssh_opts[@]}" "${g1_host}" docker logs --tail 100 \
      "${perception_container}" 2>&1 | \
      grep -Fq '[register] heartbeat ok'; then
    registration_ready=1
    break
  fi
  sleep 1
done
if [[ "${registration_ready}" != "1" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker logs --tail 200 \
    "${perception_container}" >&2
  echo "ERROR=Agent Core registration heartbeat was not observed" >&2
  false
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/core_registry_probe.py"

G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"

rollback_needed=0
trap - ERR
echo "GENERAL_NAVIGATION_G1_DEPLOY=PASS"
echo "GENERAL_NAVIGATION_COMPOSE_BACKUP=${backup_file}"
echo "GENERAL_NAVIGATION_COMPOSE_FRAGMENT=${candidate_fragment}"
echo "NOTE=Perception is registered and shadow-only; no Driver executor is connected"
