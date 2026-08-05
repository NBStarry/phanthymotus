#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
backup_name="phanthy-nav2-shadow-card1-rollback"
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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the guarded G1 N3 upgrade" >&2
  exit 2
fi

image_meta="$(docker image inspect "${NAV2_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${NAV2_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

"${script_dir}/g1-readiness.sh"
G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

current_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker container inspect --format '{{.State.Running}}|{{.Config.Image}}' ${container_name}")"
IFS='|' read -r current_running current_image <<<"${current_meta}"
if [[ "${current_running}" != "true" ]]; then
  echo "ERROR=${container_name} must be running before the N3 upgrade" >&2
  exit 1
fi
if ! ssh "${ssh_opts[@]}" "${g1_host}" test -d "${remote_maps}"; then
  echo "ERROR=remote map directory ${remote_maps} is absent" >&2
  exit 1
fi
remote_map_gid="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  stat -c '%g' "${remote_maps}")"
if [[ ! "${remote_map_gid}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=invalid map directory GID: ${remote_map_gid}" >&2
  exit 1
fi

if [[ "${current_image}" == "${NAV2_IMAGE}" ]]; then
  if ! ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
      "${container_name}" test -w /maps; then
    echo "ERROR=card2 is current but /maps is not writable; run owner-repair-map-access.sh" >&2
    exit 1
  fi
  echo "G1_NAV2_N3_UPGRADE=ALREADY_CURRENT"
  echo "NOTE=card2 is already running; no container or robot state changed"
  exit 0
fi
if [[ "${current_image}" != "phanthy-nav2:g1-humble-nav2card1" ]]; then
  echo "ERROR=unexpected current Nav2 image ${current_image}; expected card1" >&2
  exit 1
fi
if ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
    "${backup_name}" >/dev/null 2>&1; then
  echo "ERROR=rollback container ${backup_name} already exists" >&2
  exit 1
fi
remote_free_kib="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "df -Pk ${remote_maps} | tail -n 1 | awk '{print \$4}'")"
if [[ ! "${remote_free_kib}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=could not determine free space for ${remote_maps}" >&2
  exit 1
fi
required_kib=$((image_size / 1024 + 512 * 1024))
if ((remote_free_kib < required_kib)); then
  echo "ERROR=remote filesystem has ${remote_free_kib} KiB; need ${required_kib} KiB" >&2
  exit 1
fi

echo "G1_NAV2_CURRENT_IMAGE=${current_image}"
echo "G1_NAV2_TARGET_IMAGE=${NAV2_IMAGE}"
echo "G1_NAV2_TARGET_ID=${image_id}"
echo "G1_NAV2_TARGET_SIZE=${image_size}"
echo "G1_NAV2_REMOTE_MAPS=${remote_maps}"
echo "G1_NAV2_REMOTE_MAP_GID=${remote_map_gid}"
echo "G1_NAV2_REMOTE_FREE_KIB=${remote_free_kib}"
echo "G1_NAV2_ROLLBACK_CONTAINER=${backup_name}"

if [[ "${preflight_only}" == "1" ]]; then
  echo "G1_NAV2_N3_UPGRADE_PREFLIGHT=PASS"
  echo "NOTE=read-only preflight; no G1 image, file, container, or command changed"
  exit 0
fi

echo "[g1-nav2-n3] loading ${NAV2_IMAGE} before stopping card1"
docker save "${NAV2_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_image_arch="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker image inspect ${NAV2_IMAGE} --format '{{.Architecture}}'")"
if [[ "${remote_image_arch}" != "arm64" ]]; then
  echo "ERROR=remote ${NAV2_IMAGE} architecture is ${remote_image_arch}" >&2
  exit 1
fi

rollback_needed=0
rollback() {
  rc=$?
  trap - ERR
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[g1-nav2-n3] upgrade failed; restoring card1" >&2
    rollback_script="
      set -e
      if docker container inspect ${container_name} >/dev/null 2>&1; then
        docker stop --time 5 ${container_name} >/dev/null 2>&1 || true
        docker rm ${container_name} >/dev/null
      fi
      docker rename ${backup_name} ${container_name}
      docker start ${container_name}
    "
    printf -v quoted_rollback '%q' "${rollback_script}"
    ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_rollback}" || \
      echo "ERROR=automatic rollback failed; ${backup_name} is retained" >&2
  fi
  exit "${rc}"
}
trap rollback ERR

rollback_needed=1
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
ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  "${container_name}" test -w /maps

n3_probe='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  status="$(timeout 12 ros2 topic echo --once \
    --qos-reliability reliable --qos-durability transient_local \
    --field data /ubuntu/navigation/nav2/status)"
  printf "%s\n" "${status}"
  grep -Fq "\"runtime_mode\":\"mapping\"" <<<"${status}"
  grep -Fq "\"n3_ready\":true" <<<"${status}"
  echo "G1_NAV2_N3_RUNTIME=PASS"
'
printf -v quoted_n3_probe '%q' "${n3_probe}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} /bin/bash -lc ${quoted_n3_probe}"

rollback_needed=0
trap - ERR
echo "G1_NAV2_N3_UPGRADE=PASS"
echo "G1_NAV2_ROLLBACK_RETAINED=${backup_name}"
echo "NOTE=card2 remains mapping-mode and shadow-only; no Driver executor is connected"
