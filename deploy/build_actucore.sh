#!/usr/bin/env bash
# build_actucore.sh — 构建 actucore（执行模型层）镜像并推送
#
# 默认 Jetson GPU 版；--variant planar 是不含 FAST-LIVO2/CUDA 的二维导航 CPU 版。
#
# Usage:
#   ./build_actucore.sh                          # JetPack 5.11（默认），交互选源
#   ./build_actucore.sh --jp-version 6.1         # JetPack 6.1
#   ./build_actucore.sh --mirror tuna
#   ./build_actucore.sh --variant planar --mirror tuna --local
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/build_common.sh"

ENV_FILE="${SCRIPT_DIR}/.env"
if [ -f "${ENV_FILE}" ]; then
    source "${ENV_FILE}"
fi

eval "$(parse_mirror_arg "$@")"

# ── 解析参数 ─────────────────────────────────────────────────────────
JP_VERSION="5.11"
VARIANT="jetson"
LOCAL_ONLY=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --variant) VARIANT="$2"; shift 2 ;;
        --local) LOCAL_ONLY=true; shift ;;
        --jp-version) JP_VERSION="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done
if [[ "${VARIANT}" != "jetson" && "${VARIANT}" != "planar" ]]; then
    echo "Unknown variant: ${VARIANT}" >&2
    exit 1
fi

RESOURCE_CENTER_URL="${RESOURCE_CENTER_URL:-https://motus.phanthy.com}"

# If registry not configured, build locally only
PUSH_ENABLED=true
if [ -z "${REGISTRY:-}" ] || [ -z "${REGISTRY_USER:-}" ] || [ -z "${REGISTRY_PASSWORD:-}" ] || [ -z "${IMAGE_NAMESPACE:-}" ]; then
    echo "[info] Registry not configured — building locally only (no push)."
    PUSH_ENABLED=false
    REGISTRY="${REGISTRY:-local}"
    IMAGE_NAMESPACE="${IMAGE_NAMESPACE:-phanthy-motus}"
fi

DATE="$(date +%y%m%d)"
if ${LOCAL_ONLY}; then
    PUSH_ENABLED=false
    REGISTRY=local
    IMAGE_NAMESPACE=phanthy-motus
fi
COMMIT="$(git -C "${REPO_ROOT}" rev-parse --short=7 HEAD)"

# ── Jetson-only：执行模型都要 GPU，没有 CPU 变体 ──────────────────────
DOCKERFILE="${REPO_ROOT}/actucore/Dockerfile.jetson"
BUILD_CONTEXT="${REPO_ROOT}"
TAG="release.${DATE}.${COMMIT}-jetson-jp${JP_VERSION}"

BUILD_ARGS=""
# ── 根据 jp_version 选择 base image  ────────────────────────
# 表在 build_common.sh 的 jetpack_vars 里，build_perception.sh 共用同一份。
jetpack_vars "${JP_VERSION}" || exit 1
BUILD_ARGS="${BUILD_ARGS} JP_VERSION=${JP_ARG}"
CARDS_JSON='[]'
if [[ "${VARIANT}" == "planar" ]]; then
    DOCKERFILE="${REPO_ROOT}/actucore/Dockerfile.planar"
    TAG="release.${DATE}.${COMMIT}-planar"
    BUILD_ARGS=""
    ACC_ARCH="cpu"
    CARDS_JSON='["PlanarSemanticNavigation"]'
fi

# Dockerfile.jetson 基于 L4T base image —— 只有 arm64
CPU_ARCH="arm64"

FULL_IMAGE="${REGISTRY}/${IMAGE_NAMESPACE}/actucore:${TAG}"

echo "============================================"
echo "Building actucore image (${VARIANT})"
echo "Image  : ${FULL_IMAGE}"
echo "Arch   : ${ARCH} (native=${IS_ARM64})"
echo "Runs on: ${ACC_ARCH} / ${CPU_ARCH}"
echo "Push   : ${PUSH_ENABLED}"
echo "============================================"

if ${PUSH_ENABLED}; then
    echo "${REGISTRY_PASSWORD}" | docker login "${REGISTRY}" -u "${REGISTRY_USER}" --password-stdin
fi

select_mirror

# trim leading and trailing space
BUILD_ARGS="${BUILD_ARGS#${BUILD_ARGS%%[![:space:]]*}}"
BUILD_ARGS="${BUILD_ARGS%${BUILD_ARGS##*[![:space:]]}}"

BUILD_STARTED="${SECONDS}"
do_build "${DOCKERFILE}" "${BUILD_CONTEXT}" "${FULL_IMAGE}" ${BUILD_ARGS:+"${BUILD_ARGS}"}
echo "ACTUCORE_BUILD_DURATION_SEC=$((SECONDS - BUILD_STARTED))"

if ${PUSH_ENABLED}; then
    do_push "${FULL_IMAGE}"
    echo ""
    echo "Done. Image pushed: ${FULL_IMAGE}"
else
    echo ""
    echo "Done. Image built locally: ${FULL_IMAGE}"
fi

# ── 注册到 resource-center（可选）────────────────────────────────────────────
if ${PUSH_ENABLED} && [ -n "${RESOURCE_CENTER_API_KEY:-}" ]; then
    # Ask only if there is a terminal to ask on; otherwise sync (the key being
    # set is the opt-in). Test by opening /dev/tty, not with `[ -e ]`: the device
    # node exists in any container, but opening it without a controlling
    # terminal fails with ENXIO — which under `set -e` aborted the whole script
    # here, reporting a successful build as failed.
    SYNC_CONFIRM="y"
    if { : >/dev/tty; } 2>/dev/null; then
        printf "Sync to resource-center (%s)? [Y/n]: " "${RESOURCE_CENTER_URL}" >/dev/tty
        read -r SYNC_CONFIRM </dev/tty || SYNC_CONFIRM="y"
    fi
    if [[ ! "${SYNC_CONFIRM}" =~ ^[Nn] ]]; then
        echo "Registering image to resource-center (${RESOURCE_CENTER_URL})..."
        # cards 目前为空：actucore/plugins/ 还没有任何已注册的卡片（见
        # actucore/main.py 的卡片注册区注释和 actucore/README.md）。第一个卡片落地时
        # 把它加进这个数组，不要漏掉。
        HTTP_STATUS=$(curl -s -o /tmp/rc_register_resp.json -w "%{http_code}" \
            -X POST "${RESOURCE_CENTER_URL}/api/admin/register" \
            -H "Content-Type: application/json" \
            -H "x-api-key: ${RESOURCE_CENTER_API_KEY}" \
            -d "{
                \"imageRef\": \"${FULL_IMAGE}\",
                \"registryImage\": \"actucore\",
                \"tag\": \"${TAG}\",
                \"category\": \"actucore\",
                \"acc_arch\": \"${ACC_ARCH}\",
                \"cpu_arch\": \"${CPU_ARCH}\",
                \"name\": \"ActuCore\",
                \"port\": 15730,
                \"description\": \"执行模型层 — VLA 策略 / 导航 / 抓取 / locomotion / 全身控制，以 processor 卡片接入\",
                \"cards\": ${CARDS_JSON}
            }")

        if [ "${HTTP_STATUS}" = "200" ] || [ "${HTTP_STATUS}" = "201" ]; then
            echo "Registered: $(cat /tmp/rc_register_resp.json)"
        else
            echo "Warning: registration failed (HTTP ${HTTP_STATUS}): $(cat /tmp/rc_register_resp.json)"
        fi
    else
        echo "跳过同步。"
    fi
fi
