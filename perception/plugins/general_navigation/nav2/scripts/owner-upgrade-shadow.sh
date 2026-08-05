#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
backup_name="phanthy-nav2-shadow-nav2only1-rollback"
remote_maps="/home/unitree/phanthy-nav2/maps"
preflight_only="${PREFLIGHT_ONLY:-0}"

set -a
. "${nav2_dir}/source-lock.env"
set +a
NAV2_IMAGE="phanthy-nav2:g1-humble-nav2card1"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${preflight_only}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the guarded G1 shadow upgrade" >&2
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

current_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker container inspect --format '{{.State.Running}}|{{.Config.Image}}' ${container_name}")"
IFS='|' read -r current_running current_image <<<"${current_meta}"
if [[ "${current_running}" != "true" ]]; then
  echo "ERROR=${container_name} must be running before a guarded upgrade" >&2
  exit 1
fi

if ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker container inspect ${backup_name}" >/dev/null 2>&1; then
  echo "ERROR=rollback container ${backup_name} already exists; refusing to overwrite it" >&2
  exit 1
fi

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -d "${remote_maps}"; then
  echo "ERROR=remote map directory ${remote_maps} is absent" >&2
  exit 1
fi

isolation_probe='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  topics="$(ros2 topic list --no-daemon --spin-time 8)"
  if grep -Fxq "/cmd_vel" <<<"${topics}"; then
    echo "ERROR=root /cmd_vel exists" >&2
    exit 1
  fi
  shadow_info="$(timeout 8 ros2 topic info --no-daemon --spin-time 3 -v \
    /ubuntu/navigation/nav2/cmd_vel_shadow)"
  if ! grep -Fq "Subscription count: 0" <<<"${shadow_info}"; then
    printf "%s\n" "${shadow_info}" >&2
    echo "ERROR=shadow output has an external subscriber" >&2
    exit 1
  fi
  odom_status="$(timeout 12 ros2 topic echo --once \
    --qos-reliability reliable --field data \
    /ubuntu/navigation/nav2/odom_status)"
  if ! grep -Fq "\"state\": \"ready\"" <<<"${odom_status}"; then
    printf "%s\n" "${odom_status}" >&2
    echo "ERROR=native odom is not ready" >&2
    exit 1
  fi
  echo "G1_NAV2_EXISTING_SHADOW_ISOLATED=PASS"
'
printf -v quoted_isolation_probe '%q' "${isolation_probe}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} /bin/bash -lc ${quoted_isolation_probe}"

echo "G1_NAV2_CURRENT_IMAGE=${current_image}"
echo "G1_NAV2_TARGET_IMAGE=${NAV2_IMAGE}"
echo "G1_NAV2_TARGET_ID=${image_id}"
echo "G1_NAV2_TARGET_SIZE=${image_size}"
echo "G1_NAV2_ROLLBACK_CONTAINER=${backup_name}"

if [[ "${current_image}" == "${NAV2_IMAGE}" ]]; then
  G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
  echo "G1_NAV2_SHADOW_UPGRADE=ALREADY_CURRENT"
  exit 0
fi

if [[ "${preflight_only}" == "1" ]]; then
  echo "G1_NAV2_SHADOW_UPGRADE_PREFLIGHT=PASS"
  exit 0
fi

echo "[g1-nav2] loading ${NAV2_IMAGE} before stopping the current shadow"
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
    echo "[g1-nav2] upgrade failed; restoring ${backup_name}" >&2
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
      echo "ERROR=automatic rollback failed; ${backup_name} was retained" >&2
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
  --env ROS_DOMAIN_ID=42 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}" \
  --env NAV2_MODE=mapping \
  --env NAV2_MAP_YAML=/maps/map.yaml \
  --cap-drop ALL \
  --security-opt no-new-privileges:true \
  --pids-limit 512 \
  --memory 6g \
  --cpus 4 \
  --restart no \
  "${NAV2_IMAGE}"

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

rollback_needed=0
trap - ERR
echo "G1_NAV2_SHADOW_UPGRADE=PASS"
echo "G1_NAV2_ROLLBACK_RETAINED=${backup_name}"
echo "NOTE=new container remains shadow-only; no Driver executor is connected"
