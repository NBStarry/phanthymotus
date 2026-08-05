#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
mapping_backup="phanthy-nav2-shadow-n3-mapping-rollback"
remote_maps="/home/unitree/phanthy-nav2/maps"
phase="${N3_PHASE:-preflight}"
map_name="${MAP_NAME:-g1-n3-acceptance}"

set -a
. "${nav2_dir}/source-lock.env"
set +a
NAV2_IMAGE="${NAV2_N3_IMAGE:-phanthy-nav2:g1-humble-nav2card2}"

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ ! "${phase}" =~ ^(preflight|begin|save|localize|globalize|verify)$ ]]; then
  echo "ERROR=N3_PHASE must be preflight, begin, save, localize, globalize, or verify" >&2
  exit 2
fi
if [[ ! "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$ ]]; then
  echo "ERROR=MAP_NAME must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}" >&2
  exit 2
fi
if [[ "${phase}" != "preflight" && "${I_AM_G1_OWNER:-0}" != "1" ]]; then
  echo "ERROR=set I_AM_G1_OWNER=1 to authorize N3 phase ${phase}" >&2
  exit 2
fi

G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"

runtime_image="$(ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
  --format '{{.Config.Image}}' "${container_name}")"
if [[ "${runtime_image}" != "${NAV2_IMAGE}" ]]; then
  echo "ERROR=${container_name} runs ${runtime_image}; deploy ${NAV2_IMAGE} first" >&2
  exit 1
fi
remote_map_gid="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  stat -c '%g' "${remote_maps}")"
if [[ ! "${remote_map_gid}" =~ ^[0-9]+$ ]]; then
  echo "ERROR=invalid map directory GID: ${remote_map_gid}" >&2
  exit 1
fi
if ! ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
    "${container_name}" test -w /maps; then
  echo "ERROR=/maps is not writable by Nav2; run owner-repair-map-access.sh first" >&2
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

if [[ "${runtime_status}" == *'"runtime_mode":"mapping"'* ]]; then
  runtime_mode="mapping"
elif [[ "${runtime_status}" == *'"runtime_mode":"localization"'* ]]; then
  runtime_mode="localization"
else
  echo "ERROR=could not determine Nav2 runtime mode" >&2
  exit 1
fi
if [[ "${runtime_status}" != *'"n3_ready":true'* ]]; then
  echo "ERROR=Nav2 companion does not advertise n3_ready=true" >&2
  exit 1
fi

echo "G1_NAV2_N3_PHASE=${phase}"
echo "G1_NAV2_N3_MAP_NAME=${map_name}"
echo "G1_NAV2_N3_RUNTIME_MODE=${runtime_mode}"
echo "G1_NAV2_N3_MAP_GID=${remote_map_gid}"

if [[ "${phase}" == "preflight" ]]; then
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec ${container_name} find /maps -mindepth 1 -maxdepth 2 -type f -printf '%P %s bytes\n' | sort"
  echo "G1_NAV2_N3_ACCEPTANCE_PREFLIGHT=PASS"
  echo "NOTE=read-only; no map command, container change, or robot command was issued"
  exit 0
fi

if [[ "${phase}" == "begin" || "${phase}" == "save" ]]; then
  if [[ "${runtime_mode}" != "mapping" ]]; then
    echo "ERROR=N3_PHASE=${phase} requires mapping runtime" >&2
    exit 1
  fi

  runtime_python='
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    exec python3 -
  '
  printf -v quoted_runtime_python '%q' "${runtime_python}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec -i --env N3_OWNER_PHASE=${phase} --env N3_MAP_NAME=${map_name} ${container_name} /bin/bash -lc ${quoted_runtime_python}" \
    <"${nav2_dir}/tests/n3_owner_probe.py"

  if [[ "${phase}" == "begin" ]]; then
    echo "G1_NAV2_N3_ACCEPTANCE_BEGIN=PASS"
    echo "NEXT=teleoperate the robot through the mapping area, then run N3_PHASE=save with the same MAP_NAME"
  else
    echo "G1_NAV2_N3_ACCEPTANCE_SAVE=PASS"
    echo "NEXT=run N3_PHASE=localize with the same MAP_NAME while the robot is at the mapping origin"
  fi
  echo "NOTE=Nav2 remains shadow-only; no Driver executor is connected"
  exit 0
fi

if [[ "${phase}" == "localize" ]]; then
  if [[ "${runtime_mode}" != "mapping" ]]; then
    echo "ERROR=N3_PHASE=localize requires the saved mapping runtime" >&2
    exit 1
  fi
  for filename in map.yaml map.pgm map.posegraph map.data manifest.json tags.json; do
    if ! ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
        "${container_name}" test -s "/maps/${map_name}/${filename}"; then
      echo "ERROR=missing saved map artifact ${map_name}/${filename}" >&2
      exit 1
    fi
  done
  if ssh "${ssh_opts[@]}" "${g1_host}" docker container inspect \
      "${mapping_backup}" >/dev/null 2>&1; then
    echo "ERROR=rollback container ${mapping_backup} already exists" >&2
    exit 1
  fi

  rollback_needed=1
  rollback() {
    rc=$?
    trap - ERR
    if [[ "${rollback_needed}" == "1" ]]; then
      echo "[g1-nav2-n3] localization failed; restoring mapping container" >&2
      rollback_script="
        set -e
        if docker container inspect ${container_name} >/dev/null 2>&1; then
          docker stop --time 5 ${container_name} >/dev/null 2>&1 || true
          docker rm ${container_name} >/dev/null
        fi
        docker rename ${mapping_backup} ${container_name}
        docker start ${container_name}
      "
      printf -v quoted_rollback '%q' "${rollback_script}"
      ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_rollback}" || \
        echo "ERROR=automatic localization rollback failed" >&2
    fi
    exit "${rc}"
  }
  trap rollback ERR

  remote_switch="
    set -e
    docker stop --time 10 ${container_name}
    docker rename ${container_name} ${mapping_backup}
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
      --cap-drop ALL \\
      --security-opt no-new-privileges:true \\
      --pids-limit 512 \\
      --memory 6g \\
      --cpus 4 \\
      --restart no \\
      ${NAV2_IMAGE} \\
      /bin/bash -lc 'source /opt/ros/humble/setup.bash; source /nav2_ws/install/setup.bash; exec ros2 launch g1_nav2 g1_nav2.launch.py mode:=localization map_name:=${map_name} map:=/maps/${map_name}/map.yaml'
  "
  printf -v quoted_remote_switch '%q' "${remote_switch}"
  ssh "${ssh_opts[@]}" "${g1_host}" "bash -lc ${quoted_remote_switch}"

  G1_HOST="${g1_host}" "${script_dir}/audit-shadow.sh"
  ssh "${ssh_opts[@]}" "${g1_host}" docker exec \
    "${container_name}" test -w /maps
  rollback_needed=0
  trap - ERR
  echo "G1_NAV2_N3_ACCEPTANCE_LOCALIZE=PASS"
  echo "G1_NAV2_N3_MAPPING_ROLLBACK=${mapping_backup}"
  echo "NEXT=if the mapping origin is uncertain, run N3_PHASE=globalize; otherwise run N3_PHASE=verify"
  echo "NOTE=localization remains shadow-only; no Driver executor is connected"
  exit 0
fi

if [[ "${phase}" == "globalize" ]]; then
  if [[ "${runtime_mode}" != "localization" ]]; then
    echo "ERROR=N3_PHASE=globalize requires localization runtime" >&2
    exit 1
  fi
  runtime_python='
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    exec python3 -
  '
  printf -v quoted_runtime_python '%q' "${runtime_python}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec -i --env N3_OWNER_PHASE=globalize --env N3_MAP_NAME=${map_name} ${container_name} /bin/bash -lc ${quoted_runtime_python}" \
    <"${nav2_dir}/tests/n3_owner_probe.py"

  echo "G1_NAV2_N3_ACCEPTANCE_GLOBALIZE=PASS"
  echo "NEXT=slowly rotate the robot through a full turn by manual teleoperation, then run N3_PHASE=verify"
  echo "NOTE=AMCL particles were spread over the saved map; no Driver executor is connected"
  exit 0
fi

if [[ "${runtime_mode}" != "localization" ]]; then
  echo "ERROR=N3_PHASE=verify requires localization runtime" >&2
  exit 1
fi
runtime_python='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  exec python3 -
'
printf -v quoted_runtime_python '%q' "${runtime_python}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec -i --env N3_OWNER_PHASE=verify --env N3_MAP_NAME=${map_name} ${container_name} /bin/bash -lc ${quoted_runtime_python}" \
  <"${nav2_dir}/tests/n3_owner_probe.py"

echo "G1_NAV2_N3_ACCEPTANCE_VERIFY=PASS"
echo "NOTE=map reload and tag navigation were verified in shadow; no Driver executor was connected"
