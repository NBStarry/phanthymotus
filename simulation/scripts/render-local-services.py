#!/usr/bin/env python3
from __future__ import annotations

"""Render the exact simulation containers exposed in Agent Core's My Services.

The Core keeps production-equivalent Docker authority. This runtime-owned
manifest limits only what the WebUI presents; it is not a Docker security
boundary and must never be described as one.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time


SERVICE_SPECS = (
    {
        "id": "core",
        "name": "Agent Core (Simulation)",
        "category": "core",
        "container_name": "phanthymotus-sim-p0-agent-core",
        "description": "Simulation control plane and WebUI",
    },
    {
        "id": "perception",
        "name": "Perception Stack (Simulation)",
        "category": "perception",
        "container_name": "phanthymotus-sim-p0-perception",
        "mcp_url": "http://perception:15720/mcp",
        "port": 15720,
        "description": "CPU perception service for the simulation stack",
    },
    {
        "id": "simulated-g1-mujoco",
        "name": "Simulated G1 (MuJoCo)",
        "category": "driver",
        "container_name": "phanthymotus-sim-p2-g1-driver",
        "mcp_url": "http://sim-driver:15730/mcp",
        "port": 15730,
        "description": "MuJoCo G1 physics and sensor Driver",
    },
    {
        "id": "simulated-navigation-gazebo",
        "name": "Simulated Navigation (Gazebo)",
        "category": "driver",
        "container_name": "phanthymotus-sim-p3-gazebo-nav",
        "mcp_url": "http://gazebo-nav:15731/mcp",
        "port": 15731,
        "description": "Gazebo Fortress and Nav2 navigation service",
    },
)


def inspect_container(name: str):
    result = subprocess.run(
        ["docker", "inspect", name],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None
    payload = json.loads(result.stdout)
    if len(payload) != 1:
        raise RuntimeError(f"unexpected docker inspect result for {name}")
    return payload[0]


def build_manifest(now: int | None = None) -> list[dict]:
    rendered = []
    timestamp = int(time.time()) if now is None else now
    for spec in SERVICE_SPECS:
        inspected = inspect_container(spec["container_name"])
        if inspected is None:
            continue
        state = inspected.get("State", {})
        status = state.get("Status", "stopped")
        image = inspected.get("Config", {}).get("Image", "")
        if not image:
            raise RuntimeError(f"container has no configured image: {spec['container_name']}")
        entry = dict(spec)
        entry["image"] = image
        entry["last_deploy"] = {
            "image": image,
            "status": status,
            "ts": timestamp,
        }
        rendered.append(entry)
    return rendered


def write_atomic(output: Path, payload: list[dict]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{output.name}.",
        suffix=".tmp",
        dir=output.parent,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, output)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    payload = build_manifest()
    write_atomic(args.output, payload)
    print(f"local services manifest PASS output={args.output} services={len(payload)}")


if __name__ == "__main__":
    main()
