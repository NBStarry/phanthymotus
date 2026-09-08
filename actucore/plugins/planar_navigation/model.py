"""ROS-independent validation, durable map identity and serialized proposals."""

import hashlib
from contextlib import contextmanager
import json
import math
from pathlib import Path
import re
import shutil
import sqlite3
import threading
import time
import uuid


class Rejected(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def number(value, low, high, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Rejected("invalid_config", f"{field} must be a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise Rejected("invalid_config", f"{field} must be within [{low}, {high}]")
    return float(value)


DEFAULTS = {
    "namespace": "planar", "scan_topic": "/scan", "odom_topic": "/odom",
    "rgb_topic": "", "driver_status_topic": "",
    "odom_frame": "odom", "base_frame": "base_link", "map_frame": "planar_map",
    "data_dir": "/data/planar_navigation",
    "footprint": [], "execution_enabled": False,
    "max_speed": 0.2, "max_yaw_speed": 0.3, "acceleration": 0.2,
    "controller_hz": 20.0, "proposal_hz": 5.0,
    "sensor_max_age": 0.5, "command_max_age": 0.25,
    "xy_tolerance": 0.2, "yaw_tolerance": 0.1,
    "inflation_radius": 0.55, "resolution": 0.05,
    "vlm_url": "", "vlm_model": "", "vlm_api_key": "", "vlm_timeout": 20.0,
}
RANGES = {
    "max_speed": (0.01, 0.5), "max_yaw_speed": (0.05, 1.0),
    "acceleration": (0.01, 0.5), "controller_hz": (10, 50),
    "proposal_hz": (5, 20), "sensor_max_age": (0.1, 1),
    "command_max_age": (0.05, 0.25), "xy_tolerance": (0.05, 0.5),
    "yaw_tolerance": (0.03, 0.3), "inflation_radius": (0.1, 2),
    "resolution": (0.02, 0.1), "vlm_timeout": (1, 60),
}


def config(values):
    unknown = set(values) - set(DEFAULTS) - {"enabled"}
    if unknown:
        raise Rejected("invalid_config", "unknown fields: " + ",".join(sorted(unknown)))
    cfg = {**DEFAULTS, **{k: v for k, v in values.items() if k != "enabled"}}
    for key, (lo, hi) in RANGES.items():
        cfg[key] = number(cfg[key], lo, hi, key)
    for key in ("namespace", "odom_frame", "base_frame", "map_frame"):
        if not isinstance(cfg[key], str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", cfg[key]):
            raise Rejected("invalid_config", f"invalid {key}")
    if len({cfg[k] for k in ("odom_frame", "base_frame", "map_frame")}) != 3:
        raise Rejected("invalid_config", "map, odom and base frames must differ")
    for key in ("scan_topic", "odom_topic", "rgb_topic", "driver_status_topic"):
        if key in ("rgb_topic", "driver_status_topic") and cfg[key] == "":
            continue
        if not isinstance(cfg[key], str) or not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", cfg[key]):
            raise Rejected("invalid_config", f"invalid {key}")
    if not isinstance(cfg["execution_enabled"], bool):
        raise Rejected("invalid_config", "execution_enabled must be boolean")
    if cfg["execution_enabled"] and not cfg["driver_status_topic"]:
        raise Rejected("invalid_config", "physical proposal output requires Driver feedback")
    points = cfg["footprint"]
    if isinstance(points, str):
        try:
            points = json.loads(points)
        except ValueError as exc:
            raise Rejected("invalid_config", "footprint must be JSON vertices") from exc
    if not isinstance(points, list) or len(points) > 32:
        raise Rejected("invalid_config", "footprint must contain 3..32 vertices")
    if points:
        if len(points) < 3 or any(not isinstance(p, list) or len(p) != 2 for p in points):
            raise Rejected("invalid_config", "footprint must contain xy pairs")
        points = [[number(x, -3, 3, "footprint") for x in p] for p in points]
        turns = []
        for i in range(len(points)):
            a, b, c = points[i-2], points[i-1], points[i]
            turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
        if not (all(t > 0 for t in turns) or all(t < 0 for t in turns)):
            raise Rejected("invalid_config", "footprint must be strictly convex and ordered")
        for i, a in enumerate(points):
            b = points[(i+1) % len(points)]
            sides = [(b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
                     for j, p in enumerate(points) if j not in (i, (i+1) % len(points))]
            if not (all(v > 0 for v in sides) or all(v < 0 for v in sides)):
                raise Rejected("invalid_config", "footprint must not intersect itself")
        edges = [points[i][0]*points[(i+1) % len(points)][1] -
                 points[i][1]*points[(i+1) % len(points)][0] for i in range(len(points))]
        if not (all(t > 0 for t in edges) or all(t < 0 for t in edges)):
            raise Rejected("invalid_config", "footprint must enclose base origin")
    cfg["footprint"] = points
    if not isinstance(cfg["data_dir"], str) or not Path(cfg["data_dir"]).is_absolute():
        raise Rejected("invalid_config", "data_dir must be absolute")
    for key in ("vlm_url", "vlm_api_key", "vlm_model"):
        if not isinstance(cfg[key], str):
            raise Rejected("invalid_config", f"{key} must be a string")
    if cfg["vlm_url"] and not cfg["vlm_url"].startswith("https://"):
        raise Rejected("invalid_config", "VLM requires HTTPS")
    return cfg


def goal(args):
    return {k: number(args.get(k), -limit, limit, k)
            for k, limit in (("x", 10000), ("y", 10000), ("yaw", math.pi))}


def fresh(stamp, received, wall, mono, limit):
    return 0 <= mono - received <= limit and -0.05 <= wall - stamp <= limit


class Store:
    """SQLite transaction binds semantic coordinates to immutable map bytes."""
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.db = self.root / "catalog.sqlite"
        with self.connect() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS maps (
                    id TEXT PRIMARY KEY, name TEXT NOT NULL, revision TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS points (
                    id TEXT PRIMARY KEY, map_id TEXT NOT NULL, revision TEXT NOT NULL,
                    description TEXT NOT NULL, pose TEXT NOT NULL,
                    stamp REAL NOT NULL, jpeg BLOB NOT NULL);
            """)

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.db, timeout=10)
        try:
            with db:
                yield db
        finally:
            db.close()

    def check_space(self):
        if shutil.disk_usage(self.root).free < 256 * 1024 * 1024:
            raise Rejected("disk_space_low", "at least 256 MiB free space is required")

    def allocate(self):
        self.check_space()
        map_id = uuid.uuid4().hex
        folder = self.root / map_id
        folder.mkdir()
        return map_id, folder / "map"

    def digest(self, map_id):
        if not re.fullmatch(r"[a-f0-9]{32}", map_id):
            raise Rejected("map_not_found", "invalid map id")
        import yaml
        folder = self.root / map_id
        meta = folder / "map.yaml"
        if not meta.is_file() or meta.stat().st_size > 65536:
            raise Rejected("map_artifact_missing", "map YAML missing or oversized")
        content = meta.read_bytes()
        data = yaml.safe_load(content)
        if not isinstance(data, dict):
            raise Rejected("map_artifact_invalid", "invalid YAML")
        image = Path(data.get("image", ""))
        image = image if image.is_absolute() else folder / image
        if image.is_symlink() or image.resolve().parent != folder.resolve() or not image.is_file():
            raise Rejected("map_artifact_invalid", "image must be inside map directory")
        if not 16 < image.stat().st_size <= 64 * 1024 * 1024:
            raise Rejected("map_artifact_invalid", "empty or oversized map image")
        number(data.get("resolution"), 0.001, 1, "map resolution")
        # Decode once to catch truncated/invalid files before advertising a saved map.
        from PIL import Image
        with Image.open(image) as im:
            if im.width * im.height > 16_000_000:
                raise Rejected("map_artifact_invalid", "map exceeds 16 million cells")
            im.verify()
        return hashlib.sha256(content + image.read_bytes()).hexdigest()

    def finalize(self, map_id, name):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 128:
            raise Rejected("invalid_argument", "map_name must contain 1..128 characters")
        revision = self.digest(map_id)
        with self.connect() as db:
            db.execute("INSERT INTO maps VALUES (?, ?, ?)", (map_id, name.strip(), revision))
        return self.map(map_id)

    def maps(self):
        with self.connect() as db:
            return [dict(zip(("map_id", "name", "revision"), row))
                    for row in db.execute("SELECT id,name,revision FROM maps ORDER BY rowid")]

    def map(self, map_id):
        item = next((m for m in self.maps() if m["map_id"] == map_id), None)
        if not item:
            raise Rejected("map_not_found", "map id not found")
        if self.digest(map_id) != item["revision"]:
            raise Rejected("map_revision_mismatch", "saved map bytes changed")
        return {**item, "yaml": str(self.root / map_id / "map.yaml")}

    def points(self, map_id, revision):
        with self.connect() as db:
            return [dict(id=row[0], description=row[1], pose=json.loads(row[2]), stamp=row[3])
                    for row in db.execute(
                        "SELECT id,description,pose,stamp FROM points WHERE map_id=? AND revision=?",
                        (map_id, revision))]

    def capture(self, item, description, pose, stamp, jpeg):
        self.check_space()
        self.map(item["map_id"])
        point_id = uuid.uuid4().hex
        with self.connect() as db:
            if db.execute("SELECT count(*) FROM points").fetchone()[0] >= 1000:
                raise Rejected("point_limit", "delete old points before recording more")
            db.execute("INSERT INTO points VALUES (?, ?, ?, ?, ?, ?, ?)",
                       (point_id, item["map_id"], item["revision"], description,
                        json.dumps(goal(pose)), stamp, jpeg))
        return point_id

    def delete_point(self, point_id, map_id):
        with self.connect() as db:
            deleted = db.execute("DELETE FROM points WHERE id=? AND map_id=?",
                                 (point_id, map_id)).rowcount
        return bool(deleted)


class ProposalGate:
    """One lock covers invalidation, final check, sequence and publication."""
    def __init__(self, publish, cfg, clock=time.monotonic):
        self.lock = threading.RLock()
        self.publish, self.cfg, self.clock = publish, cfg, clock
        self.nav_id = None
        self.status = "idle"
        self.candidate = (0.0, 0.0)
        self.command_at = 0.0
        self.sequence = 0
        self.blocker = "not_started"
        self.healthy_since = None

    def begin(self, nav_id):
        with self.lock:
            if self.nav_id:
                raise Rejected("navigation_active", "previous task has not stopped")
            self.nav_id, self.status = nav_id, "navigating"
            self.candidate, self.command_at = (0.0, 0.0), 0
            self.healthy_since = None

    def health(self, blocker):
        with self.lock:
            now = self.clock()
            if blocker:
                changed = self.blocker != blocker
                self.healthy_since = None
                self.candidate, self.command_at = (0.0, 0.0), 0
                self.blocker = blocker
                if changed:
                    self._emit(0, 0)
            else:
                self.healthy_since = self.healthy_since if self.healthy_since is not None else now
                self.blocker = "" if now - self.healthy_since >= 1 else "recovering"

    def command(self, x, yaw):
        with self.lock:
            if not self.nav_id:
                return
            x = number(x, -1, 1, "velocity.x")
            yaw = number(yaw, -4, 4, "velocity.yaw")
            self.candidate = (max(0, min(x, self.cfg["max_speed"])),
                              max(-self.cfg["max_yaw_speed"], min(yaw, self.cfg["max_yaw_speed"])))
            self.command_at = self.clock()
            if self.candidate == (0, 0):
                self._emit(0, 0)

    def tick(self):
        with self.lock:
            if not self.nav_id:
                return
            allowed = (self.status == "navigating" and not self.blocker
                       and self.clock() - self.command_at <= self.cfg["command_max_age"])
            self._emit(*(self.candidate if allowed else (0, 0)))

    def hold(self, status, reason=""):
        with self.lock:
            self.status, self.blocker = status, reason
            self.candidate, self.command_at = (0, 0), 0
            self._emit(0, 0)

    def finish(self, status):
        with self.lock:
            self.status = status
            self._emit(0, 0)
            self.nav_id = None
            self.candidate, self.command_at = (0, 0), 0

    def _emit(self, x, yaw):
        if not self.nav_id:
            return
        self.sequence += 1
        self.publish({
            "schema": "phanthy.navigation.velocity_proposal.v1",
            "nav_id": self.nav_id, "sequence": self.sequence,
            "issued_at_unix_ms": time.time_ns() // 1_000_000, "ttl_ms": 250,
            "frame": "base_link", "nav_status": self.status,
            "velocity": {"x": x, "y": 0.0, "yaw": yaw},
            "shadow_only": True, "physical_execution": False,
            "reason": self.blocker or None,
        })
