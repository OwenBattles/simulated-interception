import math

import pytest

from interception.vector import Vector


def test_arithmetic():
    a, b = Vector(3, 4), Vector(1, 2)
    assert (a + b) == Vector(4, 6)
    assert (a - b) == Vector(2, 2)
    assert (a * 2) == Vector(6, 8)
    assert (2 * a) == Vector(6, 8)
    assert (a / 2) == Vector(1.5, 2)
    assert (-a) == Vector(-3, -4)


def test_magnitude_and_distance():
    assert Vector(3, 4).magnitude() == pytest.approx(5.0)
    assert Vector(3, 4).magnitude_squared() == pytest.approx(25.0)
    assert Vector(0, 0).dist_to(Vector(3, 4)) == pytest.approx(5.0)


def test_from_polar_round_trips_through_angle():
    v = Vector.from_polar(7.0, 0.9)
    assert v.magnitude() == pytest.approx(7.0)
    assert v.angle() == pytest.approx(0.9)


def test_truncate_clamps_only_when_over():
    assert Vector(3, 4).truncate(10.0).magnitude() == pytest.approx(5.0)
    assert Vector(3, 4).truncate(2.5).magnitude() == pytest.approx(2.5)


def test_zero_vector_normalisation_is_safe():
    """A stationary actor must not produce NaN when reorienting."""
    assert Vector(0, 0).normalize() == Vector(0, 0)
    assert Vector(0, 0).set_magnitude(5.0) == Vector(0, 0)


def test_perpendicular_and_rotate():
    v = Vector(1, 0)
    assert v.perpendicular() == Vector(0, 1)
    assert v.perpendicular().dot(v) == pytest.approx(0.0)
    rotated = v.rotate(math.pi / 2)
    assert rotated.x == pytest.approx(0.0, abs=1e-12)
    assert rotated.y == pytest.approx(1.0)


def test_copy_is_independent():
    v = Vector(1, 2)
    c = v.copy()
    c.update(x=99)
    assert v.x == 1
