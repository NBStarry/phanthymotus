#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
container_name="${NAV2_SMOKE_CONTAINER:-g1-nav2-smoke}"
domain_id="${NAV2_SMOKE_DOMAIN:-143}"
map_name="smoke-map"
tmp_root="${TMPDIR:-/tmp}"
maps_dir="$(mktemp -d "${tmp_root%/}/g1-nav2-smoke-maps.XXXXXX")"
nav2_image_override="${NAV2_IMAGE_OVERRIDE:-}"

set -a
. "${nav2_dir}/source-lock.env"
set +a
if [[ -n "${nav2_image_override}" ]]; then
  NAV2_IMAGE="${nav2_image_override}"
fi

cleanup() {
  docker stop --timeout 5 "${container_name}" >/dev/null 2>&1 || true
  maps_parent="$(cd "$(dirname "${maps_dir}")" && pwd -P)"
  expected_parent="$(cd "${tmp_root}" && pwd -P)"
  maps_basename="$(basename "${maps_dir}")"
  if [[ "${maps_parent}" == "${expected_parent}" && "${maps_basename}" == g1-nav2-smoke-maps.* ]]; then
    rm -rf -- "${maps_dir}"
  else
    echo "WARNING=refusing to clean unexpected maps directory ${maps_dir}" >&2
  fi
}
trap cleanup EXIT

if docker ps -a --format '{{.Names}}' | grep -Fxq "${container_name}"; then
  echo "ERROR=refusing to replace existing container ${container_name}" >&2
  exit 1
fi

start_runtime() {
  local mode="$1"
  local map_yaml="$2"
  local launch_suffix=""
  if [[ "${mode}" == "localization" ]]; then
    launch_suffix=" map_name:=${map_name} map:=${map_yaml}"
  fi

  docker run --rm --detach \
    --name "${container_name}" \
    --network none \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,noexec,size=256m \
    --tmpfs /root/.ros:rw,nosuid,nodev,noexec,size=64m \
    --volume "${maps_dir}:/maps" \
    --volume "${nav2_dir}/tests:/tests:ro" \
    --env "ROS_DOMAIN_ID=${domain_id}" \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}" \
    "${NAV2_IMAGE}" \
    /bin/bash -lc "source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec ros2 launch g1_nav2 g1_nav2.launch.py mode:=${mode} lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0${launch_suffix}" \
    >/dev/null

  docker exec --detach "${container_name}" /bin/bash -lc \
    'source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec python3 /tests/synthetic_inputs.py'

  local ready=0
  for _ in $(seq 1 30); do
    if docker logs "${container_name}" 2>&1 | \
      grep -Eq "lifecycle_manager_navigation.*Managed nodes are active"; then
      ready=1
      break
    fi
    sleep 1
  done

  if [[ "${ready}" != "1" ]]; then
    docker logs --tail 200 "${container_name}" >&2
    echo "ERROR=Nav2 ${mode} lifecycle did not become active" >&2
    exit 1
  fi
}

audit_runtime() {
  local phase="$1"
  local phase_upper
  local audit
  local audit_rc
  phase_upper="$(printf '%s' "${phase}" | tr '[:lower:]' '[:upper:]')"
  set +e
  audit="$({
    docker exec \
      --env "N3_PROBE_PHASE=${phase}" \
      --env "NAV2_SMOKE_MAP_NAME=${map_name}" \
      "${container_name}" /bin/bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    source /nav2_ws/install/setup.bash
    topics="$(ros2 topic list --no-daemon --spin-time 3)"
    if printf "%s\n" "$topics" | grep -Fxq "/cmd_vel"; then
      echo "ERROR=root_cmd_vel_present" >&2
      exit 1
    fi
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/cmd_vel_raw"
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/cmd_vel_shadow"
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/velocity_proposal"
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/command"
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/status"
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/cloud"
    printf "%s\n" "$topics" | grep -Fx "/ubuntu/navigation/nav2/scan"
    echo "G1_NAV2_CANVAS_CLOUD_BEGIN"
    timeout 8 ros2 topic echo --once --qos-reliability best_effort \
      --field width /ubuntu/navigation/nav2/cloud
    echo "G1_NAV2_CANVAS_CLOUD_END"
    timeout 8 ros2 topic echo --once --qos-reliability best_effort \
      --field header /ubuntu/navigation/nav2/scan
    timeout 12 ros2 topic echo --once \
      --qos-reliability reliable --qos-durability transient_local \
      --field info /map
    timeout 8 ros2 topic echo --once \
      --qos-reliability reliable --qos-durability transient_local \
      --field data /ubuntu/navigation/nav2/status
    timeout 8 ros2 topic echo --once --qos-reliability reliable --field child_frame_id /ubuntu/navigation/nav2/odom
    tf_rc=0
    timeout 8 ros2 run tf2_ros tf2_echo odom base_link || tf_rc=$?
    if [[ "$tf_rc" != "0" && "$tf_rc" != "124" ]]; then
      exit "$tf_rc"
    fi
    python3 /tests/navigation_command_probe.py
  '
  } 2>&1)"
  audit_rc=$?
  set -e
  printf '%s\n' "${audit}"
  if [[ "${audit_rc}" != "0" ]]; then
    docker logs --tail 240 "${container_name}" >&2 || true
    echo "ERROR=Nav2 ${phase} runtime audit failed with rc=${audit_rc}" >&2
    exit "${audit_rc}"
  fi

  grep -Fq "base_link" <<<"${audit}"
  grep -Fq "Translation:" <<<"${audit}"
  grep -Fq "G1_NAV2_CANVAS_CLOUD_BEGIN" <<<"${audit}"
  grep -Fq "G1_NAV2_CANVAS_CLOUD_END" <<<"${audit}"
  grep -Fq "frame_id: livox_frame" <<<"${audit}"
  grep -Fq '"shadow_only":true' <<<"${audit}"
  grep -Fq '"physical_execution":false' <<<"${audit}"
  grep -Fq "G1_NAV2_N3_${phase_upper}=PASS" <<<"${audit}"
  grep -Fq "G1_NAV2_COMMAND_BRIDGE=PASS" <<<"${audit}"
  grep -Fq "G1_NAV2_N5_PROPOSAL=PASS" <<<"${audit}"
}

stop_runtime() {
  docker stop --timeout 10 "${container_name}" >/dev/null
  for _ in $(seq 1 20); do
    if ! docker ps -a --format '{{.Names}}' | grep -Fxq "${container_name}"; then
      return
    fi
    sleep 0.25
  done
  echo "ERROR=container ${container_name} did not stop cleanly" >&2
  exit 1
}

start_runtime mapping ""
audit_runtime mapping
stop_runtime

for filename in map.yaml map.pgm map.posegraph map.data manifest.json tags.json; do
  if [[ ! -s "${maps_dir}/${map_name}/${filename}" ]]; then
    echo "ERROR=missing persisted N3 artifact ${filename}" >&2
    exit 1
  fi
done

start_runtime localization "/maps/${map_name}/map.yaml"
audit_runtime localization
docker exec -i \
  --env N3_OWNER_PHASE=globalize \
  --env "N3_MAP_NAME=${map_name}" \
  "${container_name}" /bin/bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    source /nav2_ws/install/setup.bash
    exec python3 /tests/n3_owner_probe.py
  '

echo "G1_NAV2_SMOKE_TEST=PASS"
