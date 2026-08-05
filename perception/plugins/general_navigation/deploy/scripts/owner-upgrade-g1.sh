#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
nav2_scripts="$(cd "${deploy_dir}/../nav2/scripts" && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
preflight_only="${PREFLIGHT_ONLY:-0}"
core_container="phanthy-motus-agent-core-1"
perception_container="embodied-perception"
compose_file="/opt/phanthy-motus/docker-compose.yml"
candidate_fragment="/opt/phanthy-motus/.general-navigation-service.candidate.yml"
backup_file="${compose_file}.before-general-navigation5-$(date +%Y%m%dT%H%M%S)"
rollback_container="${perception_container}-rollback"

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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the G1 Perception upgrade" >&2
  exit 2
fi

image_meta="$(docker image inspect "${GENERAL_NAVIGATION_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${GENERAL_NAVIGATION_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

core_running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}}' "${core_container}" 2>/dev/null || true)"
if [[ "${core_running}" != "true" ]]; then
  echo "ERROR=${core_container} is not running; owner must start Agent Core and rerun preflight" >&2
  exit 1
fi

G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -f "${compose_file}"; then
  echo "ERROR=${compose_file} is absent" >&2
  exit 1
fi
compose_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet

current_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}},{{.Config.Image}},{{.HostConfig.ReadonlyRootfs}},{{.HostConfig.Privileged}},{{.HostConfig.RestartPolicy.Name}}' \
  "${perception_container}")"
IFS=',' read -r current_running current_image current_read_only \
  current_privileged current_restart <<<"${current_meta}"
if [[ "${current_running}" != "true" && "${current_running}" != "false" ]]; then
  echo "ERROR=invalid current Perception running state: ${current_running}" >&2
  exit 1
fi
if [[ "${current_read_only}" != "true" && \
      "${current_read_only}" != "false" ]]; then
  echo "ERROR=invalid current Perception rootfs mode: ${current_read_only}" >&2
  exit 1
fi
if [[ "${current_privileged}" != "false" || \
      "${current_restart}" != "no" ]]; then
  echo "ERROR=unexpected current Perception runtime: ${current_meta}" >&2
  exit 1
fi

compose_probe="import yaml; d=yaml.safe_load(open('${compose_file}')) or {}; s=d.get('services',{}); v=s.get('perception'); print('absent' if v is None else v.get('image',''))"
printf -v quoted_compose_probe '%q' "${compose_probe}"
compose_image="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_compose_probe}")"
compose_service_state="present"
if [[ "${compose_image}" == "absent" ]]; then
  compose_service_state="orphan"
  compose_labels_format='{{index .Config.Labels "com.docker.compose.project"}},{{index .Config.Labels "com.docker.compose.service"}},{{index .Config.Labels "com.docker.compose.project.working_dir"}},{{index .Config.Labels "com.docker.compose.project.config_files"}}'
  printf -v quoted_compose_labels_format '%q' "${compose_labels_format}"
  printf -v quoted_perception_container '%q' "${perception_container}"
  compose_labels="$(ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker container inspect --format ${quoted_compose_labels_format} ${quoted_perception_container}")"
  if [[ "${compose_labels}" != \
        "phanthy-motus,perception,/opt/phanthy-motus,${compose_file}" ]]; then
    echo "ERROR=refusing unknown orphan Perception container: ${compose_labels}" >&2
    exit 1
  fi
elif [[ "${compose_image}" != "${current_image}" ]]; then
  echo "ERROR=compose/runtime image mismatch: ${compose_image} != ${current_image}" >&2
  exit 1
fi
if [[ "${current_image}" != "phanthy-perception:g1-general-navigation1" && \
      "${current_image}" != "phanthy-perception:g1-general-navigation2" && \
      "${current_image}" != "phanthy-perception:g1-general-navigation3" && \
      "${current_image}" != "phanthy-perception:g1-general-navigation4" && \
      "${current_image}" != "${GENERAL_NAVIGATION_IMAGE}" ]]; then
  echo "ERROR=unexpected current Perception image ${current_image}" >&2
  exit 1
fi

docker_root="$(ssh "${ssh_opts[@]}" "${g1_host}" docker info \
  --format '{{.DockerRootDir}}')"
docker_disk_line="$(ssh "${ssh_opts[@]}" "${g1_host}" df -Pk \
  "${docker_root}" | tail -n 1)"
read -r _ _ _ docker_available_kb _ <<<"${docker_disk_line}"
if [[ ! "${docker_available_kb}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=could not parse Docker free space: ${docker_disk_line}" >&2
  exit 1
fi
minimum_available_kb=$((image_size / 1024 + 256 * 1024))
if ((docker_available_kb < minimum_available_kb)); then
  echo "ERROR=remote Docker has ${docker_available_kb} KiB; need ${minimum_available_kb} KiB" >&2
  exit 1
fi

if [[ "${current_running}" == "true" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
    --env MCP_STARTUP_TIMEOUT=5 \
    --env EXPECT_BRIDGE_SUBSCRIBER=1 \
    "${perception_container}" python3 /tests/mcp_probe.py
fi

echo "GENERAL_NAVIGATION_CURRENT_IMAGE=${current_image}"
echo "GENERAL_NAVIGATION_TARGET_IMAGE=${GENERAL_NAVIGATION_IMAGE}"
echo "GENERAL_NAVIGATION_TARGET_ID=${image_id}"
echo "GENERAL_NAVIGATION_TARGET_SIZE=${image_size}"
echo "GENERAL_NAVIGATION_REMOTE_DOCKER_FREE_KIB=${docker_available_kb}"
echo "GENERAL_NAVIGATION_COMPOSE_OWNERSHIP=${compose_stat}"
echo "GENERAL_NAVIGATION_COMPOSE_SERVICE=${compose_service_state}"

if [[ "${compose_service_state}" == "present" && \
      "${current_image}" == "${GENERAL_NAVIGATION_IMAGE}" && \
      "${current_running}" == "true" && \
      "${current_read_only}" == "false" ]]; then
  REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"
  echo "GENERAL_NAVIGATION_G1_UPGRADE=ALREADY_CURRENT"
  echo "NOTE=general-navigation5 is already running with the released legacy Driver input adapter; no state changed"
  exit 0
fi
if [[ "${preflight_only}" == "1" ]]; then
  echo "GENERAL_NAVIGATION_G1_UPGRADE_PREFLIGHT=PASS"
  echo "NOTE=read-only preflight; no image, compose, container, or robot command changed"
  exit 0
fi

"${script_dir}/smoke-test.sh"
echo "[general-navigation] loading ${GENERAL_NAVIGATION_IMAGE} before compose change"
docker save "${GENERAL_NAVIGATION_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_image_arch="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${GENERAL_NAVIGATION_IMAGE}" --format '{{.Architecture}}')"
if [[ "${remote_image_arch}" != "arm64" ]]; then
  echo "ERROR=remote image architecture is ${remote_image_arch}, expected arm64" >&2
  exit 1
fi

rollback_needed=0
fragment_container="general-navigation-upgrade-fragment-$$"
rollback() {
  rc=$?
  trap - ERR
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force \
    "${fragment_container}" >/dev/null 2>&1 || true
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[general-navigation] upgrade failed; restoring compose backup" >&2
    if [[ "${compose_service_state}" == "orphan" ]]; then
      rollback_start=":"
      if [[ "${current_running}" == "true" ]]; then
        rollback_start="docker start ${perception_container} >/dev/null"
      fi
      rollback_script="set -e; docker exec ${core_container} cp -p ${backup_file} ${compose_file}; if docker container inspect ${rollback_container} >/dev/null 2>&1; then docker rm --force ${perception_container} >/dev/null 2>&1 || true; docker rename ${rollback_container} ${perception_container}; ${rollback_start}; fi; docker compose --file ${compose_file} config --quiet"
    else
      rollback_script="set -e; docker rm --force ${perception_container} >/dev/null 2>&1 || true; docker exec ${core_container} cp -p ${backup_file} ${compose_file}; docker compose --file ${compose_file} config --quiet; docker compose --file ${compose_file} up --detach --no-deps perception"
    fi
    ssh "${ssh_opts[@]}" "${g1_host}" "${rollback_script}" || \
      echo "ERROR=automatic rollback failed; backup retained at ${backup_file}" >&2
  fi
  exit "${rc}"
}
trap rollback ERR

ssh "${ssh_opts[@]}" "${g1_host}" docker exec "${core_container}" \
  cp -p "${compose_file}" "${backup_file}"
rollback_needed=1

printf -v quoted_current_image '%q' "${current_image}"
printf -v quoted_target_image '%q' "${GENERAL_NAVIGATION_IMAGE}"
if [[ "${compose_service_state}" == "orphan" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force \
    "${rollback_container}" >/dev/null 2>&1 || true
  if [[ "${current_running}" == "true" ]]; then
    ssh "${ssh_opts[@]}" "${g1_host}" docker stop --timeout 10 \
      "${perception_container}" >/dev/null
  fi
  ssh "${ssh_opts[@]}" "${g1_host}" docker rename \
    "${perception_container}" "${rollback_container}"
  ssh "${ssh_opts[@]}" "${g1_host}" docker create \
    --name "${fragment_container}" "${GENERAL_NAVIGATION_IMAGE}" >/dev/null
  ssh "${ssh_opts[@]}" "${g1_host}" docker cp \
    "${fragment_container}:/deploy/service.yml" "${candidate_fragment}"
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm \
    "${fragment_container}" >/dev/null
  merge_program="import os,sys,yaml; p='${compose_file}'; f='${candidate_fragment}'; st=os.stat(p); d=yaml.safe_load(open(p)) or {}; s=yaml.safe_load(open(f)) or {}; assert set(s)=={'perception'}; assert 'perception' not in d.get('services',{}); s['perception']['image']=sys.argv[1]; s['perception']['read_only']=False; d.setdefault('services',{}).update(s); t=p+'.general-navigation5.tmp'; h=open(t,'w'); yaml.safe_dump(d,h,default_flow_style=False,allow_unicode=True,sort_keys=False); h.close(); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
  printf -v quoted_merge_program '%q' "${merge_program}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec ${core_container} python3 -c ${quoted_merge_program} ${quoted_target_image}"
else
  update_program="import os,sys,yaml; p='${compose_file}'; st=os.stat(p); d=yaml.safe_load(open(p)) or {}; s=d.get('services',{}); assert 'perception' in s; assert s['perception'].get('image')==sys.argv[1]; s['perception']['image']=sys.argv[2]; s['perception']['read_only']=False; t=p+'.general-navigation5.tmp'; h=open(t,'w'); yaml.safe_dump(d,h,default_flow_style=False,allow_unicode=True,sort_keys=False); h.close(); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
  printf -v quoted_update_program '%q' "${update_program}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec ${core_container} python3 -c ${quoted_update_program} ${quoted_current_image} ${quoted_target_image}"
fi

updated_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
if [[ "${updated_stat}" != "${compose_stat}" ]]; then
  echo "ERROR=compose ownership changed: ${compose_stat} -> ${updated_stat}" >&2
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
  echo "ERROR=${perception_container} did not start" >&2
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
  echo "ERROR=unsafe upgraded Perception runtime: ${runtime_meta}" >&2
  false
fi

ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  --env MCP_STARTUP_TIMEOUT=30 \
  --env EXPECT_BRIDGE_SUBSCRIBER=1 \
  "${perception_container}" python3 /tests/mcp_probe.py
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/core_registry_probe.py"
REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"

rollback_needed=0
trap - ERR
echo "GENERAL_NAVIGATION_G1_UPGRADE=PASS"
echo "GENERAL_NAVIGATION_COMPOSE_BACKUP=${backup_file}"
if [[ "${compose_service_state}" == "orphan" ]]; then
  echo "GENERAL_NAVIGATION_ROLLBACK_CONTAINER=${rollback_container}"
fi
echo "NOTE=general-navigation5 is registered with the released legacy Driver input adapter; no Driver command was issued"
