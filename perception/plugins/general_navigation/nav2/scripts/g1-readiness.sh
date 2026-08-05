#!/usr/bin/env bash
set -euo pipefail

g1_host="${G1_HOST:-g1-sh-wifi}"
require_input_contract="${REQUIRE_DRIVER_INPUT_CONTRACT:-0}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"
ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

if [[ "${require_input_contract}" != "0" && "${require_input_contract}" != "1" ]]; then
  echo "ERROR=REQUIRE_DRIVER_INPUT_CONTRACT must be 0 or 1" >&2
  exit 2
fi

echo "[g1-nav2] target=${g1_host}"
ssh "${ssh_opts[@]}" "${g1_host}" hostnamectl
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker ps --format '{{.Names}}|{{.Image}}|{{.Status}}'"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' embodied-unitree-g1"

humble_probe='
  set -e
  . /opt/ros/humble/setup.bash >/dev/null 2>&1
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  topics="$(timeout 15 ros2 topic list --no-daemon --spin-time 5 -t)"
  lidar_line="$(printf "%s\n" "$topics" | grep -F "/ubuntu/lidar/cloud [")"
  if ! grep -Fq "std_msgs/msg/UInt8MultiArray" <<<"${lidar_line}"; then
    printf "%s\n" "${lidar_line}" >&2
    echo "ERROR=lidar_cloud does not advertise the UInt8MultiArray envelope" >&2
    exit 1
  fi
  printf "%s\n" "${lidar_line}"
  printf "%s\n" "$topics" | grep -F "/ubuntu/loco/state [std_msgs/msg/String]"
  timeout 15 ros2 topic info --no-daemon --spin-time 3 -v /ubuntu/lidar/cloud
  timeout 15 ros2 topic info --no-daemon --spin-time 3 -v /ubuntu/loco/state
'
printf -v quoted_probe '%q' "${humble_probe}"
ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec embodied-unitree-g1 /bin/bash -lc ${quoted_probe}"

if [[ "${require_input_contract}" == "1" ]]; then
  sample_probe='
    set -e
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    exec python3 -
  '
  printf -v quoted_sample_probe '%q' "${sample_probe}"
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker exec -i embodied-unitree-g1 /bin/bash -lc ${quoted_sample_probe}" \
    <"${nav2_dir}/tests/driver_input_contract_probe.py"
fi

echo "G1_NAV2_READINESS=PASS"
echo "NOTE=Nav2 sidecars must force UDPv4; FastDDS DEFAULT discovers /ubuntu/loco/state but does not deliver it across these containers"
if [[ "${require_input_contract}" == "1" ]]; then
  echo "NOTE=released Driver legacy inputs were validated exactly; freshness uses labelled adapter receive time because the payloads have no source stamp"
fi
