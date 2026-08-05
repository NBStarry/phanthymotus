#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"
core_dir="$(cd "${deploy_dir}/.." && pwd)"

set -a
. "${deploy_dir}/source-lock.env"
set +a

docker build \
  --platform "${TARGET_PLATFORM}" \
  --build-arg "AGENT_CORE_BASE_IMAGE=${AGENT_CORE_BASE_IMAGE}" \
  --build-arg "IMAGE_TAG=${AGENT_CORE_IMAGE_TAG}" \
  --file "${deploy_dir}/Dockerfile.navigation" \
  --tag "${AGENT_CORE_IMAGE}" \
  "${core_dir}"

docker image inspect "${AGENT_CORE_IMAGE}" \
  --format 'AGENT_CORE_IMAGE={{.Id}} architecture={{.Architecture}} size={{.Size}}'
