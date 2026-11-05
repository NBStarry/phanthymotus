"""Shared card height/radius launch transforms for both Nav2 costmaps.

Pure configuration transformation, before node startup; no ROS services or I/O.
"""
from __future__ import annotations

import math


def apply_obstacle_height_band(config: dict, minimum: float, maximum: float) -> None:
    """Validate first, then overwrite all eight leaves, including absent defaults.

    RewrittenYaml in Humble only substitutes existing leaves. Layer-level height
    keys missing from an older YAML otherwise silently retain Nav2's 0..2 default.
    Do not invent a missing costmap or observation source: reject that input.
    """
    for value in (minimum, maximum):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not -3.0 <= value <= 3.0):
            raise ValueError("obstacle height limits must be finite numbers within [-3.0, 3.0]")
    if minimum >= maximum:
        raise ValueError("obstacle minimum height must be less than maximum")
    if not isinstance(config, dict):
        raise ValueError("Nav2 parameters must be a mapping")

    targets = []
    for name in ("local_costmap", "global_costmap"):
        try:
            params = config[name][name]["ros__parameters"]
            layer = params["obstacle_layer"]
            source = layer["registered_cloud"]
            plugins = params["plugins"]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"{name} must define obstacle_layer.registered_cloud") from exc
        sources = layer.get("observation_sources") if isinstance(layer, dict) else None
        if (not isinstance(layer, dict) or not isinstance(source, dict)
                or not isinstance(plugins, list) or "obstacle_layer" not in plugins
                or sources != "registered_cloud"
                or layer.get("plugin") != "nav2_costmap_2d::ObstacleLayer"):
            raise ValueError(f"{name} must use the registered_cloud ObstacleLayer/source")
        if params.get("global_frame") != "map":
            raise ValueError(f"{name} global_frame must be map for the shared map-z height band")
        targets.extend((layer, source))
    for target in targets:
        target["min_obstacle_height"] = float(minimum)
        target["max_obstacle_height"] = float(maximum)


def footprint_inscribed_radius(config: dict, name: str) -> float:
    """Inscribed radius of the configured footprint for one costmap.

    Nav2 accepts either ``robot_radius`` (a circle) or ``footprint`` (a polygon
    as a string). The inscribed radius is the shortest distance from the origin
    to a footprint edge; an inflation radius below it produces no cost gradient
    at all, which silently turns the costmap into lethal/inscribed/free only.
    """

    try:
        params = config[name][name]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{name} must define ros__parameters") from exc
    radius = params.get("robot_radius")
    if isinstance(radius, (int, float)) and not isinstance(radius, bool):
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError(f"{name} robot_radius must be finite and positive")
        return float(radius)
    raw = params.get("footprint")
    if raw is None:
        raise ValueError(f"{name} must define robot_radius or footprint")
    points = raw
    if isinstance(raw, str):
        try:
            import json
            points = json.loads(raw)
        except ValueError as exc:
            raise ValueError(f"{name} footprint is not valid JSON") from exc
    if not isinstance(points, (list, tuple)) or len(points) < 3:
        raise ValueError(f"{name} footprint must contain at least three points")
    try:
        vertices = [(float(x), float(y)) for x, y in points]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} footprint points must be numeric pairs") from exc
    inside = point_in_polygon(0.0, 0.0, vertices)
    best = math.inf
    for index, (x1, y1) in enumerate(vertices):
        x2, y2 = vertices[(index + 1) % len(vertices)]
        best = min(best, distance_to_segment(0.0, 0.0, x1, y1, x2, y2))
    if not inside:
        # The origin outside the footprint means the polygon does not describe
        # the robot body; the distance to an edge is not an inscribed radius.
        raise ValueError(f"{name} footprint does not contain the robot origin")
    if not math.isfinite(best) or best <= 0:
        raise ValueError(f"{name} footprint inscribed radius is invalid")
    return best


def point_in_polygon(x, y, vertices):
    inside = False
    count = len(vertices)
    for index in range(count):
        x1, y1 = vertices[index]
        x2, y2 = vertices[(index + 1) % count]
        if (y1 > y) != (y2 > y):
            crossing = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < crossing:
                inside = not inside
    return inside


def distance_to_segment(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def apply_inflation_radius(config: dict, radius: float) -> None:
    """Set only the two inflation layers, including missing legacy radius leaves.

    Validate both targets before mutation. The (0, 2] limit belongs to this card,
    not Nav2; no footprint or cost scaling is changed or inferred.
    """
    if (isinstance(radius, bool) or not isinstance(radius, (int, float))
            or not 0 < radius <= 2.0 or not math.isfinite(radius)):
        raise ValueError("inflation_radius_m must be a finite number within (0, 2.0]")
    targets = []
    for name in ("local_costmap", "global_costmap"):
        try:
            params = config[name][name]["ros__parameters"]
            layer = params["inflation_layer"]
            plugins = params["plugins"]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"{name} must define inflation_layer") from exc
        if (not isinstance(layer, dict) or not isinstance(plugins, list)
                or "inflation_layer" not in plugins
                or layer.get("plugin") != "nav2_costmap_2d::InflationLayer"):
            raise ValueError(f"{name} must use the standard InflationLayer")
        inscribed = footprint_inscribed_radius(config, name)
        if float(radius) <= inscribed:
            raise ValueError(
                f"{name} inflation_radius {float(radius):.2f} m does not exceed the "
                f"footprint inscribed radius {inscribed:.2f} m; Nav2 produces no "
                "cost gradient in that case, so the costmap collapses to "
                "lethal/inscribed/free and goals inside the inscribed band look "
                "covered by obstacles"
            )
        targets.append(layer)
    for layer in targets:
        layer["inflation_radius"] = float(radius)
