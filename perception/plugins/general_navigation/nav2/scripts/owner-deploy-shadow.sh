#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
remote_root="/home/unitree/phanthy-nav2"
remote_maps="${remote_root}/maps"
preflight_only="${PREFLIGHT_ONLY:-0}"

set -a
. "${nav2_dir}/source-lock.env"
set +a

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${preflight_only}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize G1 image load and shadow container start" >&2
  exit 2
fi

image_arch="$(docker image inspect "${NAV2_IMAGE}" --format '{{.Architecture}}')"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${NAV2_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

"${script_dir}/g1-readiness.sh"

odom_probe='
  set -e
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  timeout 12 ros2 topic echo --once --qos-reliability best_effort --field data /ubuntu/loco/state
'
printf -v quoted_odom_probe '%q' "${odom_probe}"
if ! ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec phanthy-navigation-sensors-shadow /bin/bash -lc ${quoted_odom_probe}"; then
  echo "ERROR=/ubuntu/loco/state produced no UDPv4 sample; Nav2 cross-container input is not ready" >&2
  exit 1
fi

if ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker container inspect ${container_name}" >/dev/null 2>&1; then
  echo "ERROR=remote container ${container_name} already exists; refusing to replace it" >&2
  exit 1
fi

if [[ "${preflight_only}" == "1" ]]; then
  echo "G1_NAV2_OWNER_PREFLIGHT=PASS"
  exit 0
fi

echo "[g1-nav2] loading ${NAV2_IMAGE} on ${g1_host}"
docker save "${NAV2_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

ssh "${ssh_opts[@]}" "${g1_host}" mkdir -p -- "${remote_maps}"

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
  --cap-drop ALL \
  --security-opt no-new-privileges:true \
  --pids-limit 512 \
  --memory 6g \
  --cpus 4 \
  --restart no \
  "${NAV2_IMAGE}"

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

echo "G1_NAV2_SHADOW_DEPLOY=PASS"
echo "NOTE=container is mapping shadow only; no Driver executor is connected"
