#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
remote_maps="/home/unitree/phanthy-nav2/maps"
preflight_only="${PREFLIGHT_ONLY:-0}"
upgrade_stage="${NAV2_UPGRADE_STAGE:-n5}"

set -a
. "${nav2_dir}/source-lock.env"
set +a

case "${upgrade_stage}" in
  n5)
    NAV2_IMAGE="${NAV2_N5_IMAGE:-phanthy-nav2:g1-humble-nav2card3}"
    expected_current="${NAV2_N3_IMAGE:-phanthy-nav2:g1-humble-nav2card2}"
    expected_current_alt=""
    backup_name="phanthy-nav2-shadow-card2-n5-rollback"
    candidate_name="card3"
    pass_receipt="G1_NAV2_N5_PROTOCOL_UPGRADE=PASS"
    current_receipt="G1_NAV2_N5_PROTOCOL_UPGRADE=ALREADY_CURRENT"
    preflight_receipt="G1_NAV2_N5_PROTOCOL_UPGRADE_PREFLIGHT=PASS"
    ;;
  canvas-inputs)
    NAV2_IMAGE="${NAV2_CANVAS_IMAGE:-phanthy-nav2:g1-humble-nav2card4}"
    expected_current="${NAV2_N5_IMAGE:-phanthy-nav2:g1-humble-nav2card3}"
    expected_current_alt=""
    backup_name="phanthy-nav2-shadow-card3-canvas-inputs-rollback"
    candidate_name="card4"
    pass_receipt="G1_NAV2_CANVAS_INPUTS_UPGRADE=PASS"
    current_receipt="G1_NAV2_CANVAS_INPUTS_UPGRADE=ALREADY_CURRENT"
    preflight_receipt="G1_NAV2_CANVAS_INPUTS_UPGRADE_PREFLIGHT=PASS"
    ;;
  driver-inputs)
    NAV2_IMAGE="${NAV2_DRIVER_INPUT_IMAGE:-phanthy-nav2:g1-humble-nav2card5}"
    expected_current="${NAV2_CANVAS_IMAGE:-phanthy-nav2:g1-humble-nav2card4}"
    expected_current_alt="${NAV2_N5_IMAGE:-phanthy-nav2:g1-humble-nav2card3}"
    backup_name="phanthy-nav2-shadow-rollback"
    candidate_name="card5"
    pass_receipt="G1_NAV2_DRIVER_INPUTS_UPGRADE=PASS"
    current_receipt="G1_NAV2_DRIVER_INPUTS_UPGRADE=ALREADY_CURRENT"
    preflight_receipt="G1_NAV2_DRIVER_INPUTS_UPGRADE_PREFLIGHT=PASS"
    ;;
  *)
    echo "ERROR=NAV2_UPGRADE_STAGE must be n5, canvas-inputs, or driver-inputs" >&2
    exit 2
    ;;
esac

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
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the guarded N5 protocol upgrade" >&2
  exit 2
fi
if [[ "${upgrade_stage}" == "n5" && \
      "${NAV2_IMAGE}" != "phanthy-nav2:g1-humble-nav2card3" ]]; then
  echo "ERROR=N5 protocol upgrade requires card3, got ${NAV2_IMAGE}" >&2
  exit 1
fi
if [[ "${upgrade_stage}" == "canvas-inputs" && \
      "${NAV2_IMAGE}" != "phanthy-nav2:g1-humble-nav2card4" ]]; then
  echo "ERROR=canvas input upgrade requires card4, got ${NAV2_IMAGE}" >&2
  exit 1
fi
if [[ "${upgrade_stage}" == "driver-inputs" && \
      "${NAV2_IMAGE}" != "phanthy-nav2:g1-humble-nav2card5" ]]; then
  echo "ERROR=Driver contract compatibility upgrade requires card5, got ${NAV2_IMAGE}" >&2
  exit 1
fi

lidar_launch_args=""
if [[ "${upgrade_stage}" == "driver-inputs" ]]; then
  for variable in NAV2_LIDAR_X NAV2_LIDAR_Y NAV2_LIDAR_Z \
      NAV2_LIDAR_ROLL NAV2_LIDAR_PITCH NAV2_LIDAR_YAW; do
    value="${!variable:-}"
    if [[ ! "${value}" =~ ^-?[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$ ]]; then
      echo "ERROR=${variable} must be a finite decimal from the audited Driver extrinsic" >&2
      exit 2
    fi
  done
  lidar_launch_args=" lidar_x:=${NAV2_LIDAR_X} lidar_y:=${NAV2_LIDAR_Y} lidar_z:=${NAV2_LIDAR_Z} lidar_roll:=${NAV2_LIDAR_ROLL} lidar_pitch:=${NAV2_LIDAR_PITCH} lidar_yaw:=${NAV2_LIDAR_YAW}"
  if [[ -z "${NAV2_LIDAR_SOURCE:-}" ]]; then
    echo "ERROR=NAV2_LIDAR_SOURCE must identify the audited Driver extrinsic" >&2
    exit 2
  fi
fi

image_meta="$(docker image inspect "${NAV2_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${NAV2_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

require_driver_inputs=0
if [[ "${upgrade_stage}" == "driver-inputs" ]]; then
  require_driver_inputs=1
fi
REQUIRE_DRIVER_INPUT_CONTRACT="${require_driver_inputs}" \
G1_HOST="${g1_host}" \
  "${script_dir}/g1-readiness.sh"

current_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Running}},{{.Config.Image}}' "${container_name}")"
IFS=',' read -r current_running current_image <<<"${current_meta}"
current_image_id="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.Image}}' "${container_name}")"
if [[ "${current_running}" != "true" ]]; then
  echo "ERROR=${container_name} must be running before the N5 protocol upgrade" >&2
  exit 1
fi
if [[ "${current_image}" == "${NAV2_IMAGE}" && \
      "${current_image_id}" == "${image_id}" ]]; then
  REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
  echo "${current_receipt}"
  echo "NOTE=${candidate_name} is already running; no container or robot state changed"
  exit 0
fi
if [[ "${current_image}" != "${NAV2_IMAGE}" && \
      "${current_image}" != "${expected_current}" && \
      ( -z "${expected_current_alt}" || \
        "${current_image}" != "${expected_current_alt}" ) ]]; then
  echo "ERROR=unexpected current image ${current_image}; expected ${NAV2_IMAGE}, ${expected_current}, or ${expected_current_alt:-none}" >&2
  exit 1
fi

legacy_driver_input_upgrade_source_audit=0
if [[ "${upgrade_stage}" == "driver-inputs" ]]; then
  legacy_driver_input_upgrade_source_audit=1
fi
LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT="${legacy_driver_input_upgrade_source_audit}" \
G1_HOST="${g1_host}" \
  "${script_dir}/audit-shadow.sh"

rollback_present="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Status}}' "${backup_name}" 2>/dev/null || true)"
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
status_json="$(printf '%s\n' "${runtime_status}" | sed -n '/^{/{p;q;}')"
if [[ -z "${status_json}" ]]; then
  echo "ERROR=could not parse current Nav2 status" >&2
  exit 1
fi
read -r runtime_mode runtime_map navigation_status < <(
  python3 -c '
import json, sys
s = json.loads(sys.stdin.read())
print(s.get("runtime_mode", ""), s.get("active_map") or "-", s.get("status", ""))
' <<<"${status_json}"
)
if [[ "${runtime_mode}" != "mapping" && "${runtime_mode}" != "localization" ]]; then
  echo "ERROR=unsupported current runtime mode ${runtime_mode}" >&2
  exit 1
fi
if [[ "${navigation_status}" =~ ^(starting|navigating|paused)$ ]]; then
  echo "ERROR=stop active navigation ${navigation_status} before upgrading" >&2
  exit 1
fi
if [[ "${runtime_mode}" == "localization" ]]; then
  if [[ ! "${runtime_map}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]]; then
    echo "ERROR=invalid active map for localization: ${runtime_map}" >&2
    exit 1
  fi
  for filename in map.yaml map.pgm manifest.json tags.json; do
    if ! ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
        "${container_name}" test -s "/maps/${runtime_map}/${filename}"; then
      echo "ERROR=missing active map artifact ${runtime_map}/${filename}" >&2
      exit 1
    fi
  done
fi

docker_root="$(ssh "${ssh_opts[@]}" "${g1_host}" docker info \
  --format '{{.DockerRootDir}}')"
remote_free_kib="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "df -Pk ${docker_root} | tail -n 1 | awk '{print \$4}'")"
if [[ ! "${remote_free_kib}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=could not determine remote Docker free space" >&2
  exit 1
fi
required_kib=$((image_size / 1024 + 512 * 1024))
if ((remote_free_kib < required_kib)); then
  echo "ERROR=remote Docker has ${remote_free_kib} KiB; need ${required_kib} KiB" >&2
  exit 1
fi

echo "G1_NAV2_CURRENT_IMAGE=${current_image}"
echo "G1_NAV2_TARGET_IMAGE=${NAV2_IMAGE}"
echo "G1_NAV2_TARGET_ID=${image_id}"
echo "G1_NAV2_TARGET_SIZE=${image_size}"
echo "G1_NAV2_PRESERVE_MODE=${runtime_mode}"
echo "G1_NAV2_PRESERVE_MAP=${runtime_map}"
echo "G1_NAV2_REMOTE_MAP_GID=${remote_map_gid}"
echo "G1_NAV2_REMOTE_FREE_KIB=${remote_free_kib}"
echo "G1_NAV2_ROLLBACK_CONTAINER=${backup_name}"
echo "G1_NAV2_ROLLBACK_REPLACE=${rollback_present:-absent}"
if [[ "${upgrade_stage}" == "driver-inputs" ]]; then
  echo "G1_NAV2_LIDAR_EXTRINSIC_SOURCE=${NAV2_LIDAR_SOURCE}"
  echo "G1_NAV2_LIDAR_EXTRINSIC=${NAV2_LIDAR_X},${NAV2_LIDAR_Y},${NAV2_LIDAR_Z},${NAV2_LIDAR_ROLL},${NAV2_LIDAR_PITCH},${NAV2_LIDAR_YAW}"
fi

if [[ "${preflight_only}" == "1" ]]; then
  echo "${preflight_receipt}"
  echo "NOTE=read-only; no G1 image, container, map, or robot command changed"
  exit 0
fi

echo "[g1-nav2-${upgrade_stage}] loading ${NAV2_IMAGE} before stopping ${current_image}"
docker save "${NAV2_IMAGE}" | \
  ssh "${ssh_opts[@]}" "${g1_host}" docker load

remote_image_arch="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  "${NAV2_IMAGE}" --format '{{.Architecture}}')"
if [[ "${remote_image_arch}" != "arm64" ]]; then
  echo "ERROR=remote ${NAV2_IMAGE} architecture is ${remote_image_arch}" >&2
  exit 1
fi

rollback_needed=0
rollback() {
  rc=$?
  trap - ERR
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[g1-nav2-${upgrade_stage}] upgrade failed; restoring ${current_image}" >&2
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

runtime_environment=""
if [[ "${upgrade_stage}" == "driver-inputs" ]]; then
  runtime_map_env=""
  if [[ "${runtime_mode}" == "localization" ]]; then
    runtime_map_env="${runtime_map}"
  fi
  launch_command="source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec ros2 run g1_nav2 runtime_supervisor"
  runtime_environment="--env NAV2_MODE=${runtime_mode} --env NAV2_MAP_NAME=${runtime_map_env} --env NAV2_LIDAR_X=${NAV2_LIDAR_X} --env NAV2_LIDAR_Y=${NAV2_LIDAR_Y} --env NAV2_LIDAR_Z=${NAV2_LIDAR_Z} --env NAV2_LIDAR_ROLL=${NAV2_LIDAR_ROLL} --env NAV2_LIDAR_PITCH=${NAV2_LIDAR_PITCH} --env NAV2_LIDAR_YAW=${NAV2_LIDAR_YAW}"
elif [[ "${runtime_mode}" == "localization" ]]; then
  launch_command="source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec ros2 launch g1_nav2 g1_nav2.launch.py mode:=localization map_name:=${runtime_map} map:=/maps/${runtime_map}/map.yaml${lidar_launch_args}"
else
  launch_command="source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec ros2 launch g1_nav2 g1_nav2.launch.py mode:=mapping${lidar_launch_args}"
fi
remote_switch="
  set -e
  docker stop --time 10 ${container_name}
  docker rename ${container_name} ${backup_name}
  docker run --detach \\
    --name ${container_name} \\
    --network host \\
    --read-only \\
    --tmpfs /tmp:rw,nosuid,nodev,noexec,size=256m \\
    --tmpfs /root/.ros:rw,nosuid,nodev,noexec,size=64m \\
    --mount type=bind,source=${remote_maps},target=/maps \\
    --group-add ${remote_map_gid} \\
    --env ROS_DOMAIN_ID=42 \\
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \\
    --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \\
    ${runtime_environment} \\
    --cap-drop ALL \\
    --security-opt no-new-privileges:true \\
    --pids-limit 512 \\
    --memory 6g \\
    --cpus 4 \\
    --restart no \\
    ${NAV2_IMAGE} \\
    /bin/bash -lc '${launch_command}'
"
printf -v quoted_remote_switch '%q' "${remote_switch}"
if [[ -n "${rollback_present}" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force "${backup_name}" >/dev/null
fi
rollback_needed=1
ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_remote_switch}"

REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
  "${container_name}" test -w /maps

candidate_status="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} /bin/bash -lc ${quoted_status_probe}")"
printf '%s\n' "${candidate_status}"
if [[ "${candidate_status}" != *'"n5_protocol_ready":true'* || \
      "${candidate_status}" != *'"proposal_ttl_ms":250'* || \
      "${candidate_status}" != *'"proposal_subscribers":'* || \
      "${candidate_status}" != *"\"runtime_mode\":\"${runtime_mode}\""* ]]; then
  echo "ERROR=${candidate_name} did not advertise the isolated N5 protocol" >&2
  false
fi
if [[ "${runtime_mode}" == "localization" && \
      "${candidate_status}" != *"\"active_map\":\"${runtime_map}\""* ]]; then
  echo "ERROR=${candidate_name} did not preserve active map ${runtime_map}" >&2
  false
fi

if [[ "${upgrade_stage}" == "canvas-inputs" || \
      "${upgrade_stage}" == "driver-inputs" ]]; then
  canvas_probe='
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    test "$(timeout 12 ros2 topic type /ubuntu/navigation/nav2/cloud)" = \
      "sensor_msgs/msg/PointCloud2"
    lidar_info="$(timeout 12 ros2 topic info --verbose /ubuntu/lidar/cloud)"
    cloud_info="$(timeout 12 ros2 topic info --verbose /ubuntu/navigation/nav2/cloud)"
    printf "%s\n" "${lidar_info}" "${cloud_info}"
    grep -Fq "Node name: g1_canvas_pointcloud_bridge" <<<"${lidar_info}"
    grep -Fq "Endpoint type: SUBSCRIPTION" <<<"${lidar_info}"
    grep -Fq "Node name: g1_canvas_pointcloud_bridge" <<<"${cloud_info}"
    grep -Fq "Endpoint type: PUBLISHER" <<<"${cloud_info}"
    echo "G1_NAV2_CANVAS_POINTCLOUD_ADAPTER=PASS"
  '
  printf -v quoted_canvas_probe '%q' "${canvas_probe}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec ${container_name} /bin/bash -lc ${quoted_canvas_probe}"
fi

rollback_needed=0
trap - ERR
echo "${pass_receipt}"
echo "G1_NAV2_ROLLBACK_RETAINED=${backup_name}"
echo "NOTE=${candidate_name} is proposal-only and no Driver command was issued"
