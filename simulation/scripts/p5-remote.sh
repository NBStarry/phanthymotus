#!/usr/bin/env bash
set -euo pipefail
SIM_STAGE=p5 exec bash "$(dirname "$0")/p3-remote.sh" "$@"
