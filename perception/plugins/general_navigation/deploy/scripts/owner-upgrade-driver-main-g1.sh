#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
nav2_scripts="$(cd "${deploy_dir}/../nav2/scripts" && pwd)"
core_deploy="$(cd "${deploy_dir}/../../../../agent-core/deploy" && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
preflight_only="${PREFLIGHT_ONLY:-0}"
core_container="phanthy-motus-agent-core-1"
driver_container="embodied-unitree-g1"
compose_file="/opt/phanthy-motus/docker-compose.yml"
stamp="$(date +%Y%m%dT%H%M%S)"
backup_file="${compose_file}.before-driver-main-${stamp}"

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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the released Driver upgrade" >&2
  exit 2
fi

image_meta="$(docker image inspect "${GENERAL_NAVIGATION_DRIVER_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${GENERAL_NAVIGATION_DRIVER_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

core_running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}}' "${core_container}" 2>/dev/null || true)"
if [[ "${core_running}" != "true" ]]; then
  echo "ERROR=${core_container} must be running before Driver registration" >&2
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

current_exists="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format true "${driver_container}" 2>/dev/null || true)"
current_running="false"
current_image="absent"
if [[ "${current_exists}" == "true" ]]; then
  current_meta="$(remote_container_inspect \
    '{{.State.Running}}|{{.Config.Image}}' "${driver_container}")"
  IFS='|' read -r current_running current_image <<<"${current_meta}"
  if [[ "${current_running}" == "true" && \
        "${current_image}" != "${GENERAL_NAVIGATION_DRIVER_IMAGE}" ]]; then
    echo "ERROR=refusing to replace running Driver ${current_image}" >&2
    exit 1
  fi
fi

compose_probe="import yaml; d=yaml.safe_load(open('${compose_file}')) or {}; s=d.get('services',{}); v=s.get('unitree-g1'); print('absent' if v is None else v.get('image',''))"
printf -v quoted_compose_probe '%q' "${compose_probe}"
compose_image="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_compose_probe}")"
if [[ "${compose_image}" != "absent" && \
      "${compose_image}" != "${current_image}" && \
      "${compose_image}" != "${GENERAL_NAVIGATION_DRIVER_ROLLBACK_IMAGE}" && \
      "${compose_image}" != "${GENERAL_NAVIGATION_DRIVER_IMAGE}" ]]; then
  echo "ERROR=unexpected unitree-g1 compose image ${compose_image}" >&2
  exit 1
fi

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

echo "G1_DRIVER_CURRENT_IMAGE=${current_image}"
echo "G1_DRIVER_CURRENT_RUNNING=${current_running}"
echo "G1_DRIVER_COMPOSE_IMAGE=${compose_image}"
echo "G1_DRIVER_TARGET_IMAGE=${GENERAL_NAVIGATION_DRIVER_IMAGE}"
echo "G1_DRIVER_TARGET_COMMIT=${GENERAL_NAVIGATION_DRIVER_COMMIT}"
echo "G1_DRIVER_TARGET_ID=${image_id}"
echo "G1_DRIVER_TARGET_SIZE=${image_size}"
echo "G1_DRIVER_REMOTE_FREE_KIB=${remote_free_kib}"
echo "G1_DRIVER_COMPOSE_OWNERSHIP=${compose_stat}"

if [[ "${current_running}" == "true" && \
      "${current_image}" == "${GENERAL_NAVIGATION_DRIVER_IMAGE}" ]]; then
  REQUIRE_DRIVER_INPUT_CONTRACT=1 G1_HOST="${g1_host}" \
    "${nav2_scripts}/g1-readiness.sh"
  ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
    "${core_container}" python3 - \
    <"${deploy_dir}/tests/loco_registry_probe.py"
  ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
    "${core_container}" python3 - \
    <"${deploy_dir}/tests/loco_runtime_probe.py"
  echo "G1_DRIVER_MAIN_UPGRADE=ALREADY_CURRENT"
  echo "NOTE=released Driver is running and navigation execution remains unarmed"
  exit 0
fi
if [[ "${preflight_only}" == "1" ]]; then
  echo "G1_DRIVER_MAIN_UPGRADE_PREFLIGHT=PASS"
  echo "NOTE=read-only; no image, compose, container, project, or robot state changed"
  exit 0
fi

docker run --rm \
  --platform linux/arm64 \
  --network none \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=32m \
  --entrypoint python3 \
  "${GENERAL_NAVIGATION_DRIVER_IMAGE}" -c '
import yaml
cfg=yaml.safe_load(open("/work/config.yaml"))
loco=cfg["plugins"]["loco"]
assert loco["velocity_proposal_enabled"] is True
assert loco["velocity_proposal_driver_authorized"] is True
assert loco["velocity_proposal_allowed_fsm_ids"] == [500, 801]
import velocity_proposal
assert velocity_proposal.DEFAULT_VELOCITY_PROPOSAL_TOPIC == "/ubuntu/navigation/nav2/velocity_proposal"
'

echo "[g1-driver-main] loading ${GENERAL_NAVIGATION_DRIVER_IMAGE} before compose change"
docker save "${GENERAL_NAVIGATION_DRIVER_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_arch="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${GENERAL_NAVIGATION_DRIVER_IMAGE}" --format '{{.Architecture}}')"
if [[ "${remote_arch}" != "arm64" ]]; then
  echo "ERROR=remote Driver architecture is ${remote_arch}, expected arm64" >&2
  exit 1
fi

rollback_needed=0
restore_program="import os,shutil; p='${compose_file}'; b='${backup_file}'; st=os.stat(p); t=p+'.driver-main-rollback.tmp'; shutil.copyfile(b,t); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
printf -v quoted_restore_program '%q' "${restore_program}"
rollback() {
  rc=$?
  trap - ERR
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[g1-driver-main] deployment failed; restoring prior stopped Driver and compose" >&2
    if ! {
      ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force \
        "${driver_container}" >/dev/null 2>&1 || true
      ssh "${ssh_opts[@]}" "${g1_host}" \
        "docker exec ${core_container} python3 -c ${quoted_restore_program}"
      ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
        --file "${compose_file}" config --quiet
      ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
        --file "${compose_file}" up --detach --no-deps unitree-g1
    }; then
      echo "ERROR=automatic Driver rollback failed; backup retained at ${backup_file}" >&2
    fi
  fi
  exit "${rc}"
}
trap rollback ERR

ssh "${ssh_opts[@]}" "${g1_host}" cp -p "${compose_file}" "${backup_file}"
rollback_needed=1

update_program="import os,sys,yaml; p='${compose_file}'; st=os.stat(p); d=yaml.safe_load(open(p)) or {}; s=d.setdefault('services',{}); s['unitree-g1']={'image':sys.argv[1],'container_name':'${driver_container}','hostname':'ubuntu','network_mode':'host','ipc':'host','pid':'host','privileged':True,'volumes':['/dev:/dev','/opt/phanthy-motus/data:/opt/phanthy-motus/data'],'environment':['ROS_DOMAIN_ID=42','RMW_IMPLEMENTATION=rmw_fastrtps_cpp','FASTDDS_BUILTIN_TRANSPORTS=DEFAULT','NETWORK_INTERFACE=eth0','AGENT_CORE_URL=https://localhost:15678','PYTHONUNBUFFERED=1'],'logging':{'driver':'local','options':{'max-size':'10m','max-file':'3'}},'restart':'unless-stopped'}; t=p+'.driver-main.tmp'; h=open(t,'w'); yaml.safe_dump(d,h,default_flow_style=False,allow_unicode=True,sort_keys=False); h.close(); os.chmod(t,st.st_mode & 0o777); os.chown(t,st.st_uid,st.st_gid); os.replace(t,p)"
printf -v quoted_update_program '%q' "${update_program}"
printf -v quoted_target_image '%q' "${GENERAL_NAVIGATION_DRIVER_IMAGE}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${core_container} python3 -c ${quoted_update_program} ${quoted_target_image}"

updated_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%u:%g:%a' \
  "${compose_file}")"
if [[ "${updated_stat}" != "${compose_stat}" ]]; then
  echo "ERROR=compose ownership changed: ${compose_stat} -> ${updated_stat}" >&2
  false
fi
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" config --quiet
ssh "${ssh_opts[@]}" "${g1_host}" docker compose \
  --file "${compose_file}" up --detach --no-deps unitree-g1

for _ in $(seq 1 45); do
  running="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    --format '{{.State.Running}}' "${driver_container}" 2>/dev/null || true)"
  if [[ "${running}" == "true" ]]; then
    break
  fi
  sleep 1
done
if [[ "${running:-false}" != "true" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker logs --tail 260 \
    "${driver_container}" >&2 || true
  echo "ERROR=${driver_container} did not start" >&2
  false
fi

runtime_meta="$(remote_container_inspect \
  '{{.Config.Image}}|{{.Config.Hostname}}|{{.HostConfig.NetworkMode}}|{{.HostConfig.IpcMode}}|{{.HostConfig.PidMode}}|{{.HostConfig.Privileged}}' \
  "${driver_container}")"
if [[ "${runtime_meta}" != "${GENERAL_NAVIGATION_DRIVER_IMAGE}|ubuntu|host|host|host|true" ]]; then
  echo "ERROR=unexpected released Driver runtime: ${runtime_meta}" >&2
  false
fi

REQUIRE_DRIVER_INPUT_CONTRACT=1 G1_HOST="${g1_host}" \
  "${nav2_scripts}/g1-readiness.sh"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/loco_registry_probe.py"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec -i \
  "${core_container}" python3 - \
  <"${deploy_dir}/tests/loco_runtime_probe.py"

rollback_needed=0
trap - ERR
echo "G1_DRIVER_MAIN_UPGRADE=PASS"
echo "G1_DRIVER_COMPOSE_BACKUP=${backup_file}"
echo "NOTE=released Driver is registered and UNARMED; no navigation or robot command was issued"
