import math
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = (
    Path(__file__).resolve().parents[1]
    / "plugins"
    / "controlled_semantic_spatial"
    / "runtime"
    / "fast_livo2"
)
sys.path.insert(0, str(PACKAGE_ROOT))

from fast_livo2 import map_datum  # noqa: E402


def floor_points(z=-1.19, count=1000, jitter=0.02):
    return [(i * 0.1, (i % 17) * 0.1, z + (jitter if i % 2 else -jitter)) for i in range(count)]


def test_estimate_ground_z_finds_lowest_dense_slab():
    points = floor_points() + [(0.0, 0.0, 0.4)] * 50  # furniture above the floor
    estimate = map_datum.estimate_ground_z(points)
    assert estimate is not None
    assert estimate.z_m == pytest.approx(-1.19, abs=0.05)
    assert estimate.points >= 900


def test_estimate_ground_z_reports_unknown_when_too_sparse():
    assert map_datum.estimate_ground_z([(0.0, 0.0, -1.2)] * 10) is None


def test_estimate_ground_z_skips_non_finite_values():
    points = floor_points(count=400) + [(0.0, 0.0, float("nan")), (0.0, 0.0, float("inf"))]
    estimate = map_datum.estimate_ground_z(points)
    assert estimate is not None
    assert math.isfinite(estimate.z_m)


def test_check_datum_accepts_small_error_and_returns_delta():
    assert map_datum.check_datum(-1.194, -1.204) == pytest.approx(-0.01, abs=1e-6)


def test_check_datum_rejects_measured_field_offset():
    # The field case: the session sits ~0.46 m away from the saved map datum.
    with pytest.raises(map_datum.DatumError) as caught:
        map_datum.check_datum(-1.194, -0.734)
    assert "0.46" in str(caught.value) or "datum differs" in str(caught.value)


def test_check_datum_treats_unknown_as_unknown_not_as_equal():
    assert map_datum.check_datum(None, -1.19) is None
    assert map_datum.check_datum(-1.19, None) is None


def test_check_datum_honours_custom_tolerance():
    with pytest.raises(map_datum.DatumError):
        map_datum.check_datum(-1.2, -1.35, tolerance_m=0.1)
    assert map_datum.check_datum(-1.2, -1.35, tolerance_m=0.2) == pytest.approx(-0.15)


def test_floor_band_fraction_is_zero_without_datum_error():
    points = floor_points(count=2000)
    assert map_datum.floor_in_band_fraction(points, -1.19, -0.8, 0.1, stride=1) == 0.0


def test_floor_band_fraction_is_full_with_measured_datum_error():
    # 0.46 m is the measured base↔map-origin difference and pushes the whole
    # floor into [-0.8, 0.1]; this is the regression this helper guards.
    shifted = [(x, y, z + 0.46) for x, y, z in floor_points(count=2000)]
    assert map_datum.floor_in_band_fraction(shifted, -0.734, -0.8, 0.1, stride=1) == 1.0


def test_floor_band_fraction_unknown_ground_is_unknown():
    assert map_datum.floor_in_band_fraction(floor_points(), None, -0.8, 0.1) is None
