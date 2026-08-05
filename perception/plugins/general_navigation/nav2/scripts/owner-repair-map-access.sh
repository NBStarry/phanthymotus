#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
backup_name="phanthy-nav2-shadow-card2-map-access-rollback"
remote_maps="/home/unitree/phanthy-nav2/maps"
preflight_only="${PREFLIGHT_ONLY:-0}"

set -a
. "${nav2_dir}/source-lock.env"
set +a
NAV2_IMAGE="${NAV2_N3_IMAGE:-phanthy-nav2:g1-humble-nav2card2}"

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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the map-access repair" >&2
  exit 2
fi

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

runtime_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}},{{.Config.Image}}' "${container_name}")"
IFS=',' read -r runtime_running runtime_image <<<"${runtime_meta}"
if [[ "${runtime_running}" != "true" || "${runtime_image}" != "${NAV2_IMAGE}" ]]; then
  echo "ERROR=expected running ${NAV2_IMAGE}, got ${runtime_meta}" >&2
  exit 1
fi

map_stat="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  stat -c '%u,%g,%a' "${remote_maps}")"
IFS=',' read -r remote_map_uid remote_map_gid remote_map_mode <<<"${map_stat}"
if [[ ! "${remote_map_uid}" =~ ^[0-9]+$ || \
      ! "${remote_map_gid}" =~ ^[0-9]+$ || \
      ! "${remote_map_mode}" =~ ^[0-7]{3,4}$ ]]; then
  echo "ERROR=invalid map directory metadata: ${map_stat}" >&2
  exit 1
fi

status_probe='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  timeout 12 ros2 topic echo --once \
    --qos-reliability reliable --qos-durability transient_local \
    --field data /ubuntu/navigation/nav2/status
'
printf -v quoted_status_probe '%q' "${status_probe}"
runtime_status="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} /bin/bash -lc ${quoted_status_probe}")"
printf '%s\n' "${runtime_status}"
if [[ "${runtime_status}" != *'"runtime_mode":"mapping"'* || \
      "${runtime_status}" != *'"n3_ready":true'* || \
      "${runtime_status}" != *'"mapping_map":null'* ]]; then
  echo "ERROR=map-access repair requires idle card2 mapping runtime" >&2
  exit 1
fi

runtime_groups="$(ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  "${container_name}" id -G)"
map_writable=0
if ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
    "${container_name}" test -w /maps; then
  map_writable=1
fi

echo "G1_NAV2_MAP_ACCESS_IMAGE=${runtime_image}"
echo "G1_NAV2_MAP_ACCESS_HOST=${remote_map_uid}:${remote_map_gid}:${remote_map_mode}"
echo "G1_NAV2_MAP_ACCESS_GROUPS=${runtime_groups}"
echo "G1_NAV2_MAP_ACCESS_WRITABLE=${map_writable}"
echo "G1_NAV2_MAP_ACCESS_ROLLBACK=${backup_name}"

if [[ "${map_writable}" == "1" && \
      " ${runtime_groups} " == *" ${remote_map_gid} "* ]]; then
  echo "G1_NAV2_MAP_ACCESS=ALREADY_READY"
  echo "NOTE=no container or robot state changed"
  exit 0
fi
if [[ "${preflight_only}" == "1" ]]; then
  echo "G1_NAV2_MAP_ACCESS_REPAIR_PREFLIGHT=PASS"
  echo "NOTE=read-only; repair will add only host map GID ${remote_map_gid}"
  exit 0
fi
if ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    "${backup_name}" >/dev/null 2>&1; then
  echo "ERROR=rollback container ${backup_name} already exists" >&2
  exit 1
fi

rollback_needed=1
rollback() {
  rc=$?
  trap - ERR
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[g1-nav2-n3] map-access repair failed; restoring previous card2" >&2
    rollback_script="
      set -e
      if docker container inspect ${backup_name} >/dev/null 2>&1; then
        if docker container inspect ${container_name} >/dev/null 2>&1; then
          docker stop --time 5 ${container_name} >/dev/null 2>&1 || true
          docker rm ${container_name} >/dev/null
        fi
        docker rename ${backup_name} ${container_name}
        docker start ${container_name}
      elif docker container inspect ${container_name} >/dev/null 2>&1; then
        docker start ${container_name} >/dev/null 2>&1 || true
      fi
    "
    printf -v quoted_rollback '%q' "${rollback_script}"
    ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_rollback}" || \
      echo "ERROR=automatic map-access rollback failed" >&2
  fi
  exit "${rc}"
}
trap rollback ERR

ssh "${ssh_opts[@]}" "${g1_host}" docker stop --time 10 "${container_name}"
ssh "${ssh_opts[@]}" "${g1_host}" docker rename \
  "${container_name}" "${backup_name}"
ssh "${ssh_opts[@]}" "${g1_host}" docker run --detach \
  --name "${container_name}" \
  --network host \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=256m \
  --tmpfs /root/.ros:rw,nosuid,nodev,noexec,size=64m \
  --mount "type=bind,source=${remote_maps},target=/maps" \
  --group-add "${remote_map_gid}" \
  --env ROS_DOMAIN_ID=42 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}" \
  --cap-drop ALL \
  --security-opt no-new-privileges:true \
  --pids-limit 512 \
  --memory 6g \
  --cpus 4 \
  --restart no \
  "${NAV2_IMAGE}"

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
repaired_groups="$(ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  "${container_name}" id -G)"
if [[ " ${repaired_groups} " != *" ${remote_map_gid} "* ]]; then
  echo "ERROR=repaired container lacks map GID ${remote_map_gid}: ${repaired_groups}" >&2
  false
fi
ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  "${container_name}" test -w /maps

rollback_needed=0
trap - ERR
echo "G1_NAV2_MAP_ACCESS_REPAIR=PASS"
echo "G1_NAV2_MAP_ACCESS_GROUPS=${repaired_groups}"
echo "G1_NAV2_MAP_ACCESS_ROLLBACK_RETAINED=${backup_name}"
echo "NOTE=card2 was restarted in mapping mode and remains shadow-only"
