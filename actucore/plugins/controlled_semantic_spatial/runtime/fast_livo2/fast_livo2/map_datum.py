"""Map datum helpers: ground height, saved-vs-session datum check, band watchdog.

Why this exists
---------------
A saved static map stores points in the *mapping session's* frame.  Obstacle
classification later compares a point's z against a fixed height band such as
``[-0.8, 0.1] m``.  If the robot starts a new session at a different height (or
with a different pitch), the map's datum shifts relative to that band and the
floor itself can fall inside it — measured on a real map: a 0.46 m datum error
puts 100 % of the floor points into the obstacle band, which makes the planner
see obstacles everywhere.

These helpers are pure functions: no ROS, no I/O.  They let the adapter record a
datum next to a saved map, compare it against the live session, and expose a
watchdog metric that is cheap enough to sample on every diagnostic tick.
"""

from __future__ import annotations

from dataclasses import dataclass

DEFAULT_DATUM_TOLERANCE_M = 0.10
DEFAULT_GROUND_THICKNESS_M = 0.15
DEFAULT_MIN_GROUND_POINTS = 200


class DatumError(ValueError):
    """The saved map and the live session disagree about height."""


@dataclass(frozen=True)
class GroundEstimate:
    z_m: float
    points: int
    thickness_m: float


def estimate_ground_z(
    points,
    *,
    thickness_m: float = DEFAULT_GROUND_THICKNESS_M,
    min_points: int = DEFAULT_MIN_GROUND_POINTS,
):
    """Estimate the ground plane height from a point set.

    The lowest dense slab is used: take the smallest z, keep every point within
    ``thickness_m`` of it, and return the median of that slab.  Returns ``None``
    when there are too few points or the z values are unusable — callers must
    treat that as "unknown" rather than substituting a default.
    """

    zs = []
    for point in points:
        try:
            z = float(point[2])
        except (TypeError, ValueError, IndexError):
            continue
        if z != z or z in (float("inf"), float("-inf")):  # NaN / inf without math
            continue
        zs.append(z)
    if len(zs) < min_points:
        return None
    zs.sort()
    low = zs[0]
    slab = [z for z in zs if z <= low + thickness_m]
    if len(slab) < min_points:
        return None
    slab.sort()
    median = slab[len(slab) // 2]
    return GroundEstimate(
        z_m=round(median, 4),
        points=len(slab),
        thickness_m=round(slab[-1] - slab[0], 4),
    )


def datum_delta(saved_z, session_z):
    """Signed height difference ``session - saved``; ``None`` when unknown."""

    if saved_z is None or session_z is None:
        return None
    return round(float(session_z) - float(saved_z), 4)


def check_datum(saved_z, session_z, *, tolerance_m: float = DEFAULT_DATUM_TOLERANCE_M):
    """Raise when the datum moved more than the tolerance.

    ``None`` inputs mean "unknown", which is *not* an error: an old map without
    datum metadata must stay loadable, but the caller should surface
    ``datum_known=False`` instead of pretending the height matches.
    """

    if tolerance_m < 0:
        raise ValueError("tolerance_m must be non-negative")
    delta = datum_delta(saved_z, session_z)
    if delta is None:
        return None
    if abs(delta) > tolerance_m:
        raise DatumError(
            "saved map datum differs from the live session by "
            f"{delta:+.3f} m (tolerance {tolerance_m:.3f} m); the obstacle height "
            "band would misclassify the floor — rebuild or restore the map from "
            "the same start height"
        )
    return delta


def floor_in_band_fraction(points, ground_z, band_min_m, band_max_m, *, stride: int = 8):
    """Share of ground points that fall inside the obstacle band.

    A healthy map keeps this near zero; a datum error pushes it towards 1.0.
    ``stride`` subsamples so the watchdog stays cheap on large maps.
    """

    if ground_z is None:
        return None
    if band_min_m >= band_max_m:
        raise ValueError("band_min_m must be below band_max_m")
    if stride < 1:
        raise ValueError("stride must be positive")
    total = 0
    inside = 0
    for index, point in enumerate(points):
        if index % stride:
            continue
        try:
            z = float(point[2])
        except (TypeError, ValueError, IndexError):
            continue
        if abs(z - float(ground_z)) > DEFAULT_GROUND_THICKNESS_M:
            continue
        total += 1
        if band_min_m <= z <= band_max_m:
            inside += 1
    if total == 0:
        return None
    return round(inside / total, 4)
