#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav2_dir="$(cd "${script_dir}/.." && pwd)"

set -a
. "${nav2_dir}/source-lock.env"
set +a

if python3 -c 'import yaml' >/dev/null 2>&1; then
  PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="${nav2_dir}/g1_nav2" \
    python3 -m unittest discover -s "${nav2_dir}/tests" -v
else
  echo "[g1-nav2] local PyYAML unavailable; testing in pinned ROS Humble image"
  docker run --rm --pull=never --network none --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,noexec,size=32m \
    --volume "${nav2_dir}:/workspace:ro" \
    "${ROS_BASE_IMAGE}" \
    /bin/bash -lc \
    'PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/workspace/g1_nav2 python3 -m unittest discover -s /workspace/tests -v'
fi

NAV2_LIDAR_X=0 NAV2_LIDAR_Y=0 NAV2_LIDAR_Z=0 \
NAV2_LIDAR_ROLL=0 NAV2_LIDAR_PITCH=0 NAV2_LIDAR_YAW=0 \
docker compose \
  --env-file "${nav2_dir}/source-lock.env" \
  -f "${nav2_dir}/compose.nav2-shadow.yml" \
  config --quiet

echo "G1_NAV2_LOCAL_TESTS=PASS"
