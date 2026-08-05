#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
repo_root="$(cd "${script_dir}/../../../../.." && pwd)"

set -a
. "${deploy_dir}/source-lock.env"
set +a

docker build \
  --platform "${TARGET_PLATFORM}" \
  --build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}" \
  --file "${deploy_dir}/Dockerfile" \
  --tag "${GENERAL_NAVIGATION_IMAGE}" \
  "${repo_root}"

image_meta="$(docker image inspect "${GENERAL_NAVIGATION_IMAGE}" \
  --format '{{.Architecture}}|{{.Id}}|{{.Size}}')"
IFS='|' read -r image_arch image_id image_size <<<"${image_meta}"
if [[ "${image_arch}" != "arm64" ]]; then
  echo "ERROR=${GENERAL_NAVIGATION_IMAGE} architecture is ${image_arch}, expected arm64" >&2
  exit 1
fi

echo "GENERAL_NAVIGATION_IMAGE=${GENERAL_NAVIGATION_IMAGE}"
echo "GENERAL_NAVIGATION_IMAGE_ID=${image_id}"
echo "GENERAL_NAVIGATION_IMAGE_SIZE=${image_size}"
echo "GENERAL_NAVIGATION_BUILD=PASS"
