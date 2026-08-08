import pytest

from interception.collision import closest_approach, swept_hit
from interception.vector import Vector


def test_crossing_paths_meet_mid_step():
    miss, t = closest_approach(
        Vector(0, 0), Vector(10, 0), Vector(10, 0), Vector(0, 0)
    )
    assert miss == pytest.approx(0.0, abs=1e-9)
    assert t == pytest.approx(0.5)


def test_parallel_motion_holds_separation():
    """Equal velocities mean zero relative motion: separation is constant."""
    miss, t = closest_approach(Vector(0, 0), Vector(10, 0), Vector(0, 5), Vector(10, 5))
    assert miss == pytest.approx(5.0)
    assert t == pytest.approx(0.0)


def test_closest_approach_clamps_to_the_step():
    """Still closing at the end of the step: answer is the endpoint, not the future."""
    miss, t = closest_approach(Vector(0, 0), Vector(1, 0), Vector(100, 0), Vector(99, 0))
    assert t == pytest.approx(1.0)
    assert miss == pytest.approx(98.0)


def test_swept_test_catches_a_pass_through_that_endpoints_miss():
    """
    Regression for tunnelling.

    A body crossing in front of a static obstacle starts 5.1 m away and ends
    5.1 m away, so sampling only the endpoints reports no contact -- but it
    passes within 1 m mid-step, well inside the 2 m capture radius.
    """
    p1, p2 = Vector(0, 0), Vector(10, 0)
    q1, q2 = Vector(5, 1), Vector(5, 1)

    assert p1.dist_to(q1) > 2.0
    assert p2.dist_to(q2) > 2.0

    hit, miss = swept_hit(p1, p2, q1, q2, combined_radius=2.0)
    assert hit
    assert miss == pytest.approx(1.0)


def test_swept_test_reports_miss_distance_on_a_clean_miss():
    hit, miss = swept_hit(
        Vector(0, 0), Vector(10, 0), Vector(5, 9), Vector(5, 9), combined_radius=2.0
    )
    assert not hit
    assert miss == pytest.approx(9.0)
