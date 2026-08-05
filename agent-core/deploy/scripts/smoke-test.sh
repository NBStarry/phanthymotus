#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deploy_dir="$(cd "${script_dir}/.." && pwd)"

set -a
. "${deploy_dir}/source-lock.env"
set +a

docker run --rm \
  --platform "${TARGET_PLATFORM}" \
  --network none \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,noexec,size=64m \
  --tmpfs /work/resource:rw,nosuid,nodev,noexec,size=64m \
  --env PYTHONDONTWRITEBYTECODE=1 \
  --entrypoint /bin/bash \
  "${AGENT_CORE_IMAGE}" -lc '
    set -e
    test "$(cat /work/VERSION)" = "g1-general-navigation1"
    grep -Fq "fieldSchema.type === '\''integer'\''" /work/web/js/canvas.js
    grep -Fq "field?.style.display === '\''none'\''" /work/web/js/canvas.js
    PYTHONPATH=/work/src /work/.venv/bin/python -c "
import navigation_execution
import topic_action_routing
from api import config, mcp_manage
assert callable(navigation_execution.call_with_execution_lease)
assert callable(topic_action_routing.resolve_topic_action_routes)
assert callable(config._do_start_project)
assert callable(mcp_manage.mcp_call_tool)
"
  '

echo "AGENT_CORE_NAVIGATION_SMOKE=PASS"
