#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
target_mode="${NAV2_TARGET_MODE:-}"
requested_map="${MAP_NAME:-}"
preflight_only="${PREFLIGHT_ONLY:-0}"
container_name="phanthy-nav2-shadow"
backup_name="phanthy-nav2-shadow-rollback"
remote_maps="/home/unitree/phanthy-nav2/maps"

set -a
. "${nav2_dir}/source-lock.env"
set +a

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${target_mode}" != "mapping" && "${target_mode}" != "localization" ]]; then
  echo "ERROR=NAV2_TARGET_MODE must be mapping or localization" >&2
  exit 2
fi
if [[ "${preflight_only}" != "0" && "${preflight_only}" != "1" ]]; then
  echo "ERROR=PREFLIGHT_ONLY must be 0 or 1" >&2
  exit 2
fi
if [[ "${preflight_only}" != "1" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize the Nav2 runtime mode switch" >&2
  exit 2
fi
if [[ -n "${requested_map}" && \
      ! "${requested_map}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]]; then
  echo "ERROR=MAP_NAME must be a plain 1-64 character map name" >&2
  exit 2
fi

for variable in NAV2_LIDAR_X NAV2_LIDAR_Y NAV2_LIDAR_Z \
    NAV2_LIDAR_ROLL NAV2_LIDAR_PITCH NAV2_LIDAR_YAW; do
  value="${!variable:-}"
  if [[ ! "${value}" =~ ^-?[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$ ]]; then
    echo "ERROR=${variable} must be a finite decimal from the audited Driver extrinsic" >&2
    exit 2
  fi
done
lidar_launch_args=" lidar_x:=${NAV2_LIDAR_X} lidar_y:=${NAV2_LIDAR_Y} lidar_z:=${NAV2_LIDAR_Z} lidar_roll:=${NAV2_LIDAR_ROLL} lidar_pitch:=${NAV2_LIDAR_PITCH} lidar_yaw:=${NAV2_LIDAR_YAW}"

REQUIRE_DRIVER_INPUT_CONTRACT=1 G1_HOST="${g1_host}" \
  "${script_dir}/g1-readiness.sh"

container_format='{{.State.Running}}|{{.Config.Image}}|{{.Image}}'
printf -v quoted_container_format '%q' "${container_format}"
current_meta="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker container inspect --format ${quoted_container_format} ${container_name}")"
IFS='|' read -r current_running current_image current_image_id <<<"${current_meta}"
if [[ "${current_running}" != "true" ]]; then
  echo "ERROR=${container_name} must be running before switching modes" >&2
  exit 1
fi
if [[ "${current_image}" != "${NAV2_IMAGE}" ]]; then
  echo "ERROR=${container_name} runs ${current_image}; expected ${NAV2_IMAGE}" >&2
  exit 1
fi
remote_image_id="$(ssh "${ssh_opts[@]}" "${g1_host}" docker image inspect \
  --format '{{.Id}}' "${NAV2_IMAGE}")"
if [[ "${current_image_id}" != "${remote_image_id}" ]]; then
  echo "ERROR=running card5 image ID does not match the current ${NAV2_IMAGE} tag" >&2
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
read -r current_mode active_map mapping_map navigation_status < <(
  python3 -c '
import json, sys
s = json.loads(sys.stdin.read())
print(
    s.get("runtime_mode", ""),
    s.get("active_map") or "-",
    s.get("mapping_map") or "-",
    s.get("status", ""),
)
' <<<"${status_json}"
)
if [[ "${current_mode}" != "mapping" && "${current_mode}" != "localization" ]]; then
  echo "ERROR=unsupported current runtime mode ${current_mode}" >&2
  exit 1
fi
if [[ "${navigation_status}" =~ ^(starting|navigating|paused)$ ]]; then
  echo "ERROR=stop active navigation ${navigation_status} before switching modes" >&2
  exit 1
fi
if [[ "${mapping_map}" != "-" ]]; then
  echo "ERROR=stop active mapping ${mapping_map} from the Canvas before switching modes" >&2
  exit 1
fi

target_map="-"
if [[ "${target_mode}" == "localization" ]]; then
  target_map="${requested_map:-${active_map}}"
  if [[ ! "${target_map}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]]; then
    echo "ERROR=MAP_NAME is required when no saved active map is available" >&2
    exit 2
  fi
  for filename in map.yaml map.pgm manifest.json tags.json; do
    if ! ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
        "${container_name}" test -s "/maps/${target_map}/${filename}"; then
      echo "ERROR=missing saved map artifact ${target_map}/${filename}" >&2
      exit 1
    fi
  done
fi

if [[ "${current_mode}" == "${target_mode}" && \
      ( "${target_mode}" != "localization" || "${active_map}" == "${target_map}" ) ]]; then
  REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
  echo "G1_NAV2_RUNTIME_MODE=${current_mode}"
  echo "G1_NAV2_RUNTIME_SWITCH=ALREADY_CURRENT"
  echo "NOTE=read-only; no Nav2 container, map, Driver, or robot state changed"
  exit 0
fi

if ! ssh "${ssh_opts[@]}" "${g1_host}" test -d "${remote_maps}"; then
  echo "ERROR=remote map directory ${remote_maps} is absent" >&2
  exit 1
fi
remote_map_gid="$(ssh "${ssh_opts[@]}" "${g1_host}" stat -c '%g' "${remote_maps}")"
if [[ ! "${remote_map_gid}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=invalid map directory GID: ${remote_map_gid}" >&2
  exit 1
fi
rollback_present="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.State.Status}}' "${backup_name}" 2>/dev/null || true)"

echo "G1_NAV2_CURRENT_MODE=${current_mode}"
echo "G1_NAV2_TARGET_MODE=${target_mode}"
echo "G1_NAV2_TARGET_MAP=${target_map}"
echo "G1_NAV2_IMAGE=${current_image}"
echo "G1_NAV2_IMAGE_ID=${current_image_id}"
echo "G1_NAV2_REMOTE_MAP_GID=${remote_map_gid}"
echo "G1_NAV2_ROLLBACK_CONTAINER=${backup_name}"
echo "G1_NAV2_ROLLBACK_REPLACE=${rollback_present:-absent}"
echo "G1_NAV2_LIDAR_EXTRINSIC_SOURCE=${NAV2_LIDAR_SOURCE}"
echo "G1_NAV2_LIDAR_EXTRINSIC=${NAV2_LIDAR_X},${NAV2_LIDAR_Y},${NAV2_LIDAR_Z},${NAV2_LIDAR_ROLL},${NAV2_LIDAR_PITCH},${NAV2_LIDAR_YAW}"

if [[ "${preflight_only}" == "1" ]]; then
  echo "G1_NAV2_RUNTIME_SWITCH_PREFLIGHT=PASS"
  echo "NOTE=read-only; no Nav2 container, map, Driver, or robot state changed"
  exit 0
fi

runtime_map_env=""
if [[ "${target_mode}" == "localization" ]]; then
  runtime_map_env="${target_map}"
fi
launch_command="source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec ros2 run g1_nav2 runtime_supervisor"

rollback_needed=0
rollback() {
  rc=$?
  trap - ERR
  if [[ "${rollback_needed}" == "1" ]]; then
    echo "[g1-nav2-mode] switch failed; restoring ${current_mode}" >&2
    rollback_script="
      set -e
      if docker container inspect ${backup_name} >/dev/null 2>&1; then
        if docker container inspect ${container_name} >/dev/null 2>&1; then
          docker stop --time 5 ${container_name} >/dev/null 2>&1 || true
          docker rm ${container_name} >/dev/null
        fi
        docker rename ${backup_name} ${container_name}
      fi
      docker start ${container_name} >/dev/null
    "
    printf -v quoted_rollback '%q' "${rollback_script}"
    ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_rollback}" || \
      echo "ERROR=automatic rollback failed; inspect ${container_name} and ${backup_name}" >&2
  fi
  exit "${rc}"
}
trap rollback ERR

if [[ -n "${rollback_present}" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" docker rm --force "${backup_name}" >/dev/null
fi
rollback_needed=1
ssh "${ssh_opts[@]}" "${g1_host}" docker stop --time 10 "${container_name}" >/dev/null
ssh "${ssh_opts[@]}" "${g1_host}" docker rename "${container_name}" "${backup_name}"

remote_run="
  set -e
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
    --env NAV2_MODE=${target_mode} \\
    --env NAV2_MAP_NAME=${runtime_map_env} \\
    --env NAV2_LIDAR_X=${NAV2_LIDAR_X} \\
    --env NAV2_LIDAR_Y=${NAV2_LIDAR_Y} \\
    --env NAV2_LIDAR_Z=${NAV2_LIDAR_Z} \\
    --env NAV2_LIDAR_ROLL=${NAV2_LIDAR_ROLL} \\
    --env NAV2_LIDAR_PITCH=${NAV2_LIDAR_PITCH} \\
    --env NAV2_LIDAR_YAW=${NAV2_LIDAR_YAW} \\
    --cap-drop ALL \\
    --security-opt no-new-privileges:true \\
    --pids-limit 512 \\
    --memory 6g \\
    --cpus 4 \\
    --restart no \\
    ${NAV2_IMAGE} \\
    /bin/bash -lc '${launch_command}'
"
printf -v quoted_remote_run '%q' "${remote_run}"
ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_remote_run}"

REQUIRE_N5_PROTOCOL=1 G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
ssh "${ssh_opts[@]}" "${g1_host}" docker exec "${container_name}" test -w /maps

candidate_status="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec ${container_name} /bin/bash -lc ${quoted_status_probe}")"
printf '%s\n' "${candidate_status}"
if [[ "${candidate_status}" != *"\"runtime_mode\":\"${target_mode}\""* || \
      "${candidate_status}" != *'"n5_protocol_ready":true'* ]]; then
  echo "ERROR=card5 did not become ready in ${target_mode} mode" >&2
  false
fi
if [[ "${target_mode}" == "localization" && \
      "${candidate_status}" != *"\"active_map\":\"${target_map}\""* ]]; then
  echo "ERROR=card5 did not load map ${target_map}" >&2
  false
fi

rollback_needed=0
trap - ERR
echo "G1_NAV2_RUNTIME_MODE=${target_mode}"
echo "G1_NAV2_RUNTIME_SWITCH=PASS"
echo "G1_NAV2_ROLLBACK_RETAINED=${backup_name}"
if [[ "${target_mode}" == "mapping" ]]; then
  echo "NEXT=start the Canvas project, select start_mapping, enter a new MAP_NAME, and execute"
else
  echo "NEXT=start the Canvas project and use the saved map ${target_map}"
fi
echo "NOTE=card5 remains proposal-only; no Driver or robot motion command was issued"
