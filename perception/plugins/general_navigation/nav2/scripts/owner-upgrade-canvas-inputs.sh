#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_UPGRADE_STAGE=canvas-inputs \
  exec "${script_dir}/owner-upgrade-n5-protocol.sh" "$@"
