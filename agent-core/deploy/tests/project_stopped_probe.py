#!/usr/bin/env python3
"""Require Agent Core's canvas project to be stopped before replacement."""

from __future__ import annotations

import json
import pathlib
import ssl
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


def main() -> None:
    headers = {}
    token = access_token()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request(
        "https://127.0.0.1:15678/api/config/project-running",
        headers=headers,
    )
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(request, timeout=5, context=context) as response:
        payload = json.load(response)
    if payload != {"running": False}:
        raise SystemExit(
            "ERROR=Agent Core canvas project is running; stop it manually in "
            "the UI before upgrading Driver, Core, Perception, or canvas wiring"
        )
    print("AGENT_CORE_PROJECT_STOPPED=PASS")


if __name__ == "__main__":
    main()
