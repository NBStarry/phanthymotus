#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-}"
case "$ACTION" in
  build|push|build-and-push) ;;
  *) printf 'usage: %s {build|push|build-and-push}\n' "$0" >&2; exit 2 ;;
esac

RUNTIME_ROOT="${RUNTIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SOURCE_DIR="${PLATFORM_SOURCE_DIR:?set PLATFORM_SOURCE_DIR to a clean 4paradigm/phanthymotus main checkout}"
REVISION="${PLATFORM_REVISION:-$(git -C "$SOURCE_DIR" rev-parse HEAD)}"
SHORT="${REVISION:0:12}"
REGISTRY="${TCR_REGISTRY:-bj-warehouse.tencentcloudcr.com/phanthy-motus}"
TAG="sim-main-${SHORT}-amd64"
ROS_BASE_IMAGE="${ROS_BASE_IMAGE:-phanthymotus-sim/ros-base:humble-amd64}"
BUILD_PROXY="${BUILD_PROXY:-}"

CORE_IMAGE="${REGISTRY}/core:${TAG}"
PERCEPTION_IMAGE="${REGISTRY}/perception:${TAG}"
ACTUCORE_IMAGE="${REGISTRY}/actucore:${TAG}"

require_source() {
  [[ "$(git -C "$SOURCE_DIR" rev-parse HEAD)" == "$REVISION" ]]
  [[ -z "$(git -C "$SOURCE_DIR" status --porcelain)" ]]
  [[ "$(git -C "$SOURCE_DIR" rev-parse refs/remotes/origin/main)" == "$REVISION" ]]
}

prepare_context() {
  CONTEXT="$(mktemp -d)"
  trap 'rm -rf "$CONTEXT"' EXIT
  git -C "$SOURCE_DIR" archive "$REVISION" | tar -x -C "$CONTEXT"
  mkdir -p "$CONTEXT/simulation/config" "$CONTEXT/agent-core/tests"
  cp "$RUNTIME_ROOT/config/perception-p0.yaml" "$CONTEXT/simulation/config/"
  cp "$RUNTIME_ROOT/../agent-core/tests/test_local_services.py" "$CONTEXT/agent-core/tests/"
}

build_one() {
  local dockerfile="$1" image="$2"
  DOCKER_BUILDKIT=1 docker build --network host \
    --build-arg HTTP_PROXY="$BUILD_PROXY" --build-arg HTTPS_PROXY="$BUILD_PROXY" \
    --build-arg SOURCE_REVISION="$REVISION" --build-arg ROS_BASE_IMAGE="$ROS_BASE_IMAGE" \
    -f "$dockerfile" -t "$image" "$CONTEXT"
}

build() {
  require_source
  prepare_context
  build_one "$RUNTIME_ROOT/docker/agent-core.Dockerfile" "$CORE_IMAGE"
  build_one "$RUNTIME_ROOT/docker/perception.Dockerfile" "$PERCEPTION_IMAGE"
  build_one "$RUNTIME_ROOT/docker/actucore.Dockerfile" "$ACTUCORE_IMAGE"
  docker image inspect "$CORE_IMAGE" "$PERCEPTION_IMAGE" "$ACTUCORE_IMAGE" >/dev/null
  printf 'BUILD PASS revision=%s\n%s\n%s\n%s\n' "$REVISION" "$CORE_IMAGE" "$PERCEPTION_IMAGE" "$ACTUCORE_IMAGE"
}

push() {
  docker push "$CORE_IMAGE"
  docker push "$PERCEPTION_IMAGE"
  docker push "$ACTUCORE_IMAGE"
  printf 'PUSH PASS\n%s\n%s\n%s\n' "$CORE_IMAGE" "$PERCEPTION_IMAGE" "$ACTUCORE_IMAGE"
}

case "$ACTION" in
  build) build ;;
  push) push ;;
  build-and-push) build; push ;;
esac
