#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
nav2_scripts="$(cd "${deploy_dir}/../nav2/scripts" && pwd)"
core_deploy="$(cd "${deploy_dir}/../../../../agent-core/deploy" && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
preflight_only="${PREFLIGHT_ONLY:-0}"
core_container="phanthy-motus-agent-core-1"
perception_container="embodied-perception"
nav2_container="phanthy-nav2-shadow"
compose_file="/opt/phanthy-motus/docker-compose.yml"
candidate_fragment="/opt/phanthy-motus/.navigation2-service.candidate.yml"
stamp="$(date +%Y%m%dT%H%M%S)"
backup_file="${compose_file}.before-navigation2-${stamp}"
perception_rollback="${perception_container}-rollback"
nav2_rollback="${nav2_container}-compose-rollback"

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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the Navigation 2 deployment" >&2
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
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${core_deploy}/tests/project_stopped_probe.py"

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -f "${compose_file}"; then
  echo "ERROR=${compose_file} is absent" >&2
  exit 1
fi
compose_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet

perception_exists="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format true "${perception_container}" 2>/dev/null || true)"
nav2_exists="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format true "${nav2_container}" 2>/dev/null || true)"
if [[ "${perception_exists}" != "${nav2_exists}" ]]; then
  echo "ERROR=Navigation 2 container pair is partial: perception=${perception_exists:-false}, nav2=${nav2_exists:-false}" >&2
  exit 1
fi

current_running="false"
current_image="absent"
current_image_id="absent"
current_read_only="false"
current_restart="absent"
nav2_running="false"
nav2_current_image="absent"
nav2_current_image_id="absent"
nav2_current_restart="absent"
if [[ "${perception_exists}" == "true" ]]; then
  current_meta="$(remote_container_inspect \
    '{{.State.Running}}|{{.Config.Image}}|{{.Image}}|{{.HostConfig.ReadonlyRootfs}}|{{.HostConfig.Privileged}}|{{.HostConfig.RestartPolicy.Name}}' \
    "${perception_container}")"
  IFS='|' read -r current_running current_image current_image_id \
    current_read_only current_privileged current_restart <<<"${current_meta}"
  if [[ "${current_privileged}" != "false" || \
        ( "${current_restart}" != "no" && "${current_restart}" != "unless-stopped" ) ]]; then
    echo "ERROR=unexpected current Perception runtime: ${current_meta}" >&2
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

  nav2_meta="$(remote_container_inspect \
    '{{.State.Running}}|{{.Config.Image}}|{{.Image}}|{{.HostConfig.ReadonlyRootfs}}|{{.HostConfig.Privileged}}|{{.HostConfig.RestartPolicy.Name}}' \
    "${nav2_container}")"
  IFS='|' read -r nav2_running nav2_current_image nav2_current_image_id \
    nav2_read_only nav2_privileged nav2_current_restart <<<"${nav2_meta}"
  if [[ "${nav2_current_image}" != "${GENERAL_NAVIGATION_NAV2_IMAGE}" || \
        "${nav2_read_only}" != "true" || \
        "${nav2_privileged}" != "false" || \
        ( "${nav2_current_restart}" != "no" && "${nav2_current_restart}" != "unless-stopped" ) ]]; then
    echo "ERROR=unexpected current Nav2 runtime: ${nav2_meta}" >&2
    exit 1
  fi
fi

compose_probe="import yaml; d=yaml.safe_load(open('${compose_file}')) or {}; s=d.get('services',{}); p=s.get('perception'); n=s.get('nav2'); print(('absent' if p is None and n is None else 'partial' if p is None or n is None else 'present')+'|'+('' if p is None else p.get('image',''))+'|'+('' if n is None else n.get('image','')))"
printf -v quoted_compose_probe '%q' "${compose_probe}"
compose_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_compose_probe}")"
IFS='|' read -r compose_service_state compose_image compose_nav2_image \
  <<<"${compose_meta}"
if [[ "${compose_service_state}" == "partial" ]]; then
  echo "ERROR=compose must declare both perception and nav2, or neither" >&2
  exit 1
fi
if [[ "${compose_service_state}" == "present" && \
      ( "${compose_image}" != "${current_image}" || \
        "${compose_nav2_image}" != "${nav2_current_image}" ) ]]; then
  echo "ERROR=compose/runtime image mismatch: ${compose_meta}" >&2
  exit 1
fi

if [[ "${perception_exists}" == "true" ]]; then
  perception_labels="$(remote_container_inspect \
    '{{index .Config.Labels "com.docker.compose.project"}}|{{index .Config.Labels "com.docker.compose.service"}}' \
    "${perception_container}")"
  nav2_labels="$(remote_container_inspect \
    '{{index .Config.Labels "com.docker.compose.project"}}|{{index .Config.Labels "com.docker.compose.service"}}' \
    "${nav2_container}")"
  if [[ "${compose_service_state}" == "present" ]]; then
    if [[ "${perception_labels}" != "phanthy-motus|perception" || \
          "${nav2_labels}" != "phanthy-motus|nav2" ]]; then
      echo "ERROR=refusing unexpected managed container labels: ${perception_labels}, ${nav2_labels}" >&2
      exit 1
    fi
  elif [[ "${perception_labels}" != "phanthy-motus|perception" || \
          "${nav2_labels}" != "nav2|nav2-shadow" ]]; then
    echo "ERROR=refusing unknown orphan containers: ${perception_labels}, ${nav2_labels}" >&2
    exit 1
  fi
fi

nav2_target_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${GENERAL_NAVIGATION_NAV2_IMAGE}" --format '{{.Architecture}},{{.Id}}')"
IFS=',' read -r nav2_target_arch nav2_target_id <<<"${nav2_target_meta}"
if [[ "${nav2_target_arch}" != "arm64" ]]; then
  echo "ERROR=${GENERAL_NAVIGATION_NAV2_IMAGE} on G1 is ${nav2_target_arch}, expected arm64" >&2
  exit 1
fi

for rollback_name in "${perception_rollback}" "${nav2_rollback}"; do
  if ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
      "${rollback_name}" >/dev/null 2>&1; then
    echo "ERROR=rollback container ${rollback_name} already exists; inspect it before deployment" >&2
    exit 1
  fi
done

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

if [[ "${current_running}" == "true" && "${nav2_running}" == "true" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
    --env MCP_STARTUP_TIMEOUT=5 \
    --env EXPECT_BRIDGE_SUBSCRIBER=1 \
    "${perception_container}" python3 /tests/mcp_probe.py
  G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"
fi

echo "NAVIGATION2_CURRENT_PERCEPTION_IMAGE=${current_image}"
echo "NAVIGATION2_CURRENT_NAV2_IMAGE=${nav2_current_image}"
echo "NAVIGATION2_TARGET_PERCEPTION_IMAGE=${GENERAL_NAVIGATION_IMAGE}"
echo "NAVIGATION2_TARGET_PERCEPTION_ID=${image_id}"
echo "NAVIGATION2_TARGET_NAV2_IMAGE=${GENERAL_NAVIGATION_NAV2_IMAGE}"
echo "NAVIGATION2_TARGET_NAV2_ID=${nav2_target_id}"
echo "NAVIGATION2_REMOTE_DOCKER_FREE_KIB=${docker_available_kb}"
echo "NAVIGATION2_COMPOSE_OWNERSHIP=${compose_stat}"
echo "NAVIGATION2_COMPOSE_SERVICE=${compose_service_state}"

if [[ "${compose_service_state}" == "present" && \
      "${current_running}" == "true" && "${nav2_running}" == "true" && \
      "${current_image_id}" == "${image_id}" && \
      "${nav2_current_image_id}" == "${nav2_target_id}" && \
      "${current_read_only}" == "false" && \
      "${current_restart}" == "unless-stopped" && \
      "${nav2_current_restart}" == "unless-stopped" ]]; then
  REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${nav2_scripts}/audit-shadow.sh"
  echo "NAVIGATION2_G1_DEPLOY=ALREADY_CURRENT"
  echo "NOTE=Navigation 2 and Nav2 are already managed by the formal Perception Compose project; no state changed"
  exit 0
fi
if [[ "${preflight_only}" == "1" ]]; then
  echo "NAVIGATION2_G1_DEPLOY_PREFLIGHT=PASS"
  echo "NOTE=read-only preflight; no image, compose, container, or robot command changed"
  exit 0
fi

"${script_dir}/smoke-test.sh"
echo "[navigation2] loading ${GENERAL_NAVIGATION_IMAGE} before Compose migration"
docker save "${GENERAL_NAVIGATION_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_image_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${GENERAL_NAVIGATION_IMAGE}" --format '{{.Architecture}},{{.Id}}')"
IFS=',' read -r remote_image_arch remote_image_id <<<"${remote_image_meta}"
if [[ "${remote_image_arch}" != "arm64" || "${remote_image_id}" != "${image_id}" ]]; then
  echo "ERROR=remote Perception image does not match local candidate: ${remote_image_meta}" >&2
  exit 1
fi

rollback_needed=0
perception_renamed=0
nav2_renamed=0
fragment_container="navigation2-compose-fragment-$$"
rollback() {
  rc=$?
  trap - ERR
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force \
    "${fragment_container}" >/dev/null 2>&1 || true
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[navigation2] deployment failed; restoring Compose and retained containers" >&2
    rollback_script="set -e"
    if [[ "${perception_exists}" != "true" || "${perception_renamed}" == "1" ]]; then
      rollback_script+="; docker rm --force ${perception_container} >/dev/null 2>&1 || true"
    fi
    if [[ "${nav2_exists}" != "true" || "${nav2_renamed}" == "1" ]]; then
      rollback_script+="; docker rm --force ${nav2_container} >/dev/null 2>&1 || true"
    fi
    rollback_script+="; docker exec ${core_container} cp -p ${backup_file} ${compose_file}"
    if [[ "${perception_renamed}" == "1" ]]; then
      rollback_script+="; docker rename ${perception_rollback} ${perception_container}"
    fi
    if [[ "${nav2_renamed}" == "1" ]]; then
      rollback_script+="; docker rename ${nav2_rollback} ${nav2_container}"
    fi
    rollback_script+="; docker compose --file ${compose_file} config --quiet"
    if [[ "${current_running}" == "true" ]]; then
      rollback_script+="; docker start ${perception_container} >/dev/null"
    fi
    if [[ "${nav2_running}" == "true" ]]; then
      rollback_script+="; docker start ${nav2_container} >/dev/null"
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

ssh "${ssh_opts[@]}" "${g1_host}" docker create \
  --name "${fragment_container}" "${GENERAL_NAVIGATION_IMAGE}" >/dev/null
ssh "${ssh_opts[@]}" "${g1_host}" docker cp \
  "${fragment_container}:/deploy/service.yml" "${candidate_fragment}"
ssh "${ssh_opts[@]}" "${g1_host}" docker rm \
  "${fragment_container}" >/dev/null

if [[ "${perception_exists}" == "true" ]]; then
  if [[ "${current_running}" == "true" ]]; then
    ssh "${ssh_opts[@]}" "${g1_host}" docker stop --timeout 10 \
      "${perception_container}" >/dev/null
  fi
  if [[ "${nav2_running}" == "true" ]]; then
    ssh "${ssh_opts[@]}" "${g1_host}" docker stop --timeout 10 \
      "${nav2_container}" >/dev/null
  fi
  ssh "${ssh_opts[@]}" "${g1_host}" docker rename \
    "${perception_container}" "${perception_rollback}"
  perception_renamed=1
  ssh "${ssh_opts[@]}" "${g1_host}" docker rename \
    "${nav2_container}" "${nav2_rollback}"
  nav2_renamed=1
fi

merge_program="import os,sys,yaml; p='${compose_file}'; f='${candidate_fragment}'; st=os.stat(p); d=yaml.safe_load(open(p)) or {}; s=yaml.safe_load(open(f)) or {}; assert set(s)=={'perception','nav2'}; assert s['nav2'].get('image')==sys.argv[2]; s['perception']['image']=sys.argv[1]; s['perception']['read_only']=False; assert s['perception'].get('restart')=='unless-stopped'; assert s['nav2'].get('restart')=='unless-stopped'; d.setdefault('services',{}).update(s); t=p+'.navigation2.tmp'; h=open(t,'w'); yaml.safe_dump(d,h,default_flow_style=False,allow_unicode=True,sort_keys=False); h.close(); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
printf -v quoted_merge_program '%q' "${merge_program}"
printf -v quoted_target_image '%q' "${GENERAL_NAVIGATION_IMAGE}"
printf -v quoted_nav2_image '%q' "${GENERAL_NAVIGATION_NAV2_IMAGE}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_merge_program} ${quoted_target_image} ${quoted_nav2_image}"

updated_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
if [[ "${updated_stat}" != "${compose_stat}" ]]; then
  echo "ERROR=compose ownership changed: ${compose_stat} -> ${updated_stat}" >&2
  false
fi
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" up --detach perception

for container in "${nav2_container}" "${perception_container}"; do
  container_running="false"
  for _ in $(seq 1 45); do
    container_running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
      --format '{{.State.Running}}' "${container}" 2>/dev/null || true)"
    if [[ "${container_running}" == "true" ]]; then
      break
    fi
    sleep 1
  done
  if [[ "${container_running}" != "true" ]]; then
    ssh "${ssh_opts[@]}" "${g1_host}" docker logs --tail 200 \
      "${container}" >&2 || true
    echo "ERROR=${container} did not start" >&2
    false
  fi
done

runtime_meta="$(remote_container_inspect \
  '{{.Config.Image}}|{{.Image}}|{{.HostConfig.ReadonlyRootfs}}|{{.HostConfig.Privileged}}|{{.HostConfig.PidMode}}|{{.HostConfig.RestartPolicy.Name}}|{{index .Config.Labels "com.docker.compose.project"}}|{{index .Config.Labels "com.docker.compose.service"}}' \
  "${perception_container}")"
IFS='|' read -r runtime_image runtime_image_id runtime_read_only \
  runtime_privileged runtime_pid_mode runtime_restart runtime_project \
  runtime_service <<<"${runtime_meta}"
if [[ "${runtime_image}" != "${GENERAL_NAVIGATION_IMAGE}" || \
      "${runtime_image_id}" != "${image_id}" || \
      "${runtime_read_only}" != "false" || \
      "${runtime_privileged}" != "false" || \
      -n "${runtime_pid_mode}" || \
      "${runtime_restart}" != "unless-stopped" || \
      "${runtime_project}" != "phanthy-motus" || \
      "${runtime_service}" != "perception" ]]; then
  echo "ERROR=unsafe upgraded Perception runtime: ${runtime_meta}" >&2
  false
fi

nav2_runtime_meta="$(remote_container_inspect \
  '{{.Config.Image}}|{{.Image}}|{{.HostConfig.ReadonlyRootfs}}|{{.HostConfig.Privileged}}|{{.HostConfig.RestartPolicy.Name}}|{{index .Config.Labels "com.docker.compose.project"}}|{{index .Config.Labels "com.docker.compose.service"}}' \
  "${nav2_container}")"
IFS='|' read -r nav2_runtime_image nav2_runtime_image_id \
  nav2_runtime_read_only nav2_runtime_privileged nav2_runtime_restart \
  nav2_runtime_project nav2_runtime_service <<<"${nav2_runtime_meta}"
if [[ "${nav2_runtime_image}" != "${GENERAL_NAVIGATION_NAV2_IMAGE}" || \
      "${nav2_runtime_image_id}" != "${nav2_target_id}" || \
      "${nav2_runtime_read_only}" != "true" || \
      "${nav2_runtime_privileged}" != "false" || \
      "${nav2_runtime_restart}" != "unless-stopped" || \
      "${nav2_runtime_project}" != "phanthy-motus" || \
      "${nav2_runtime_service}" != "nav2" ]]; then
  echo "ERROR=unsafe upgraded Nav2 runtime: ${nav2_runtime_meta}" >&2
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
echo "NAVIGATION2_G1_DEPLOY=PASS"
echo "NAVIGATION2_COMPOSE_BACKUP=${backup_file}"
if [[ "${perception_exists}" == "true" ]]; then
  echo "NAVIGATION2_PERCEPTION_ROLLBACK=${perception_rollback}"
  echo "NAVIGATION2_NAV2_ROLLBACK=${nav2_rollback}"
fi
echo "NOTE=Navigation 2 and Nav2 are now owned by the formal Perception Compose project; no Driver command was issued"
