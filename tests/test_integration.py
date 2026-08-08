"""
Timestep-independence of the integrator.

The pre-refactor sim advanced with ``pos += vel`` and ``vel += acc``, so
every speed and acceleration was implicitly per-frame: running at 120 Hz
made the vehicles fly twice as fast. These tests pin the fix.
"""

import pytest

from interception.actor import Actor
from interception.vector import Vector


def make_actor(fake_state, **kwargs):
    params = dict(
        mass=5.0,
        max_speed=120.0,
        max_force=1000.0,
        hit_radius_m=2.0,
    )
    params.update(kwargs)
    return Actor(fake_state, **params)


def integrate_for(actor, force, duration_s, dt):
    for _ in range(round(duration_s / dt)):
        actor.integrate(force, dt)
    return actor


def test_coasting_distance_is_independent_of_timestep(fake_state, place):
    """Zero force: displacement is v*T at any dt, exactly."""
    results = []
    for dt in (1 / 60, 1 / 240, 1 / 1000):
        actor = place(make_actor(fake_state), pos=(0, 0), vel=(50, -25))
        integrate_for(actor, Vector(), duration_s=1.0, dt=dt)
        results.append(actor.pos)

    for pos in results:
        assert pos.x == pytest.approx(50.0, rel=1e-9)
        assert pos.y == pytest.approx(-25.0, rel=1e-9)


def test_velocity_under_constant_force_is_independent_of_timestep(fake_state, place):
    """Semi-implicit Euler integrates velocity exactly for constant force."""
    force = Vector(500.0, 0.0)  # 500 N / 5 kg = 100 m/s^2
    speeds = []
    for dt in (1 / 60, 1 / 240, 1 / 1000):
        actor = place(make_actor(fake_state), pos=(0, 0), vel=(0, 0))
        integrate_for(actor, force, duration_s=1.0, dt=dt)
        speeds.append(actor.vel.x)

    for speed in speeds:
        assert speed == pytest.approx(100.0, rel=1e-9)


def test_position_error_under_constant_force_shrinks_with_timestep(fake_state, place):
    """
    Position carries O(dt) truncation error, so it cannot match exactly --
    but refining the timestep must reduce the error toward the analytic
    x = 0.5*a*t^2.
    """
    force = Vector(500.0, 0.0)
    analytic_x = 0.5 * 100.0 * 1.0**2

    errors = []
    for dt in (1 / 60, 1 / 600):
        actor = place(make_actor(fake_state), pos=(0, 0), vel=(0, 0))
        integrate_for(actor, force, duration_s=1.0, dt=dt)
        errors.append(abs(actor.pos.x - analytic_x))

    assert errors[1] < errors[0] / 5


def test_force_is_clamped_to_the_airframe_limit(fake_state, place):
    actor = place(make_actor(fake_state, max_force=1000.0), pos=(0, 0), vel=(0, 0))
    actor.integrate(Vector(1e6, 1e6), dt=1 / 60)
    assert actor.steering_force.magnitude() == pytest.approx(1000.0)
    assert actor.acc.magnitude() == pytest.approx(200.0)  # 1000 N / 5 kg


def test_speed_is_clamped_to_max_speed(fake_state, place):
    actor = place(make_actor(fake_state, max_speed=120.0), pos=(0, 0), vel=(0, 0))
    integrate_for(actor, Vector(1000.0, 0.0), duration_s=10.0, dt=1 / 60)
    assert actor.vel.magnitude() == pytest.approx(120.0)


def test_delta_v_accumulates_control_effort(fake_state, place):
    actor = place(make_actor(fake_state), pos=(0, 0), vel=(0, 0))
    assert actor.delta_v_mps == 0.0
    integrate_for(actor, Vector(500.0, 0.0), duration_s=0.5, dt=1 / 60)
    # 100 m/s^2 held for 0.5 s
    assert actor.delta_v_mps == pytest.approx(50.0, rel=1e-9)


def test_bounds_reflect_velocity_and_clamp_position(fake_state, place):
    actor = place(make_actor(fake_state), pos=(10, 500), vel=(-100, 0))
    actor.integrate(Vector(), dt=1.0)  # overshoots the left wall
    assert actor.enforce_bounds()
    assert actor.pos.x == 0.0
    assert actor.vel.x > 0


def test_reorient_survives_a_stationary_actor(fake_state, place):
    """A zero-velocity actor must keep a valid body frame, not NaN."""
    actor = place(make_actor(fake_state), pos=(0, 0), vel=(1, 0))
    actor.vel = Vector(0, 0)
    actor.reorient()
    assert actor.forward_vec.magnitude() == pytest.approx(1.0)
    assert actor.side_vec.dot(actor.forward_vec) == pytest.approx(0.0)
