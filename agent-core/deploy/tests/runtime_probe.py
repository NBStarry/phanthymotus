#!/usr/bin/env python3
"""Verify the upgraded Agent Core API and trusted navigation code are live."""

from __future__ import annotations

import json
import os
import pathlib
import ssl
import sys
import time
import urllib.error
import urllib.request


def access_token() -> str:
    path = pathlib.Path("/opt/phanthy-motus/.env")
    if not path.exists():
        return ""
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if line.startswith("ACCESS_TOKEN=") and not line.startswith("#"):
            return line.split("=", 1)[1].strip()
    return ""


def get_json(path: str) -> dict:
    headers = {}
    token = access_token()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request(
        f"https://127.0.0.1:15678/api/{path}", headers=headers
    )
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        return json.load(response)


def main() -> None:
    expected = os.environ["EXPECTED_IMAGE_TAG"]
    assert pathlib.Path("/work/VERSION").read_text().strip() == expected
    canvas_js = pathlib.Path("/work/web/js/canvas.js").read_text()
    assert "fieldSchema.type === 'integer'" in canvas_js
    assert "field?.style.display === 'none'" in canvas_js
    sys.path.insert(0, "/work/src")
    import navigation_execution
    import topic_action_routing

    assert callable(navigation_execution.call_with_execution_lease)
    assert callable(topic_action_routing.resolve_topic_action_routes)
    deadline = time.monotonic() + 30.0
    last_error = None
    while time.monotonic() < deadline:
        try:
            verify = get_json("auth/verify")
            assert verify.get("valid") is True, verify
            project = get_json("config/project-running")
            assert project == {"running": False}, project
            print(json.dumps({"version": expected, "project": project}))
            print("AGENT_CORE_NAVIGATION_RUNTIME=PASS")
            return
        except (OSError, urllib.error.URLError, AssertionError) as exc:
            last_error = exc
            time.sleep(0.5)
    raise RuntimeError(f"Agent Core did not become ready: {last_error}")


if __name__ == "__main__":
    main()
