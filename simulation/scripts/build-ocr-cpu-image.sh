#!/usr/bin/env bash
# Run on the target x86 build machine; wheels must already be downloaded there.
set -euo pipefail
SOURCE_DIR="${1:?usage: build-ocr-cpu-image.sh OCR_CHECKOUT REMOTE_WHEEL_DIR IMAGE_TAG}"
WHEELS="${2:?missing wheel directory}"
IMAGE="${3:?missing output image tag}"
REV=7f87c173cee45354686527d9a684fbcbef67fe2c
BASE_IMAGE="${BASE_IMAGE:-sha256:cbdedb4a6b3f4c2ab8b392d339a536b291aa6af39ed47758e1e7098456ff1047}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$(docker image inspect --format '{{.Architecture}}' "$BASE_IMAGE")" == amd64 ]]
git -C "$SOURCE_DIR" cat-file -e "$REV^{commit}"
CONTEXT="$(mktemp -d)"
trap 'rm -rf "$CONTEXT"' EXIT
git -C "$SOURCE_DIR" diff "$REV^" "$REV" -- \
  perception/plugins/ocr.py perception/plugins/ocr_runtime.py perception/utils/model_downloader.py \
  > "$CONTEXT/ocr-cpu-only.patch"
[[ -s "$CONTEXT/ocr-cpu-only.patch" ]]
git -C "$SOURCE_DIR" archive "$REV" perception/tools/verify_ocr_cpu.py | tar -x -C "$CONTEXT"
cp "$WHEELS/rapidocr-3.9.1-py3-none-any.whl" "$CONTEXT/"
cp "$WHEELS/pyvips_binary-8.18.6-cp37-abi3-manylinux_2_28_x86_64.whl" "$CONTEXT/"
docker build --network host --build-arg BASE_IMAGE="$BASE_IMAGE" \
  --build-arg HTTP_PROXY="${BUILD_PROXY:-}" --build-arg HTTPS_PROXY="${BUILD_PROXY:-}" \
  -f "$ROOT/docker/perception-ocr-cpu.Dockerfile" -t "$IMAGE" "$CONTEXT"
docker image inspect --format '{{.Id}}' "$IMAGE"
