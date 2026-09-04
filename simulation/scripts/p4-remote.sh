#!/usr/bin/env bash
set -euo pipefail
SIM_STAGE=p4 exec bash "$(dirname "$0")/p3-remote.sh" "$@"
