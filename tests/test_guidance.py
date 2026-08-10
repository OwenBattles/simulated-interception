"""
Behavioural tests for the guidance laws.

These pin the properties that make PN worth having, not just that the code
runs: LOS rate driven to zero, a straight terminal course, and lower control
effort than pursuit against the same geometry.
"""

import math

import pytest

from interception.agent import Agent
from interception.guidance import (
    AugmentedProportionalNavigation,
    LAWS,
    ProportionalNavigation,
    PurePursuit,
    engagement_geometry,
    los_normal,
    make_guidance,
)
from interception.params import GuidanceParams, default_interceptor
from interception.vector import Vector


class ConstantVelocityTarget:
    """Non-manoeuvring target, so PN's assumptions hold exactly."""

    def __init__(self, pos, vel):
        self.pos = pos
        self.vel = vel
        self.acc = Vector()
        self.hit_radius_m = 1.0

    def step(self, dt):
        self.pos = self.pos + self.vel * dt


def fly(fake_state, law, target, steps=6000, dt=1 / 60, nav_constant=4.0):
    """Run one interceptor against one target; return the flight history."""
    # A very large world keeps the wall-reflection logic out of the way.
    fake_state.width = fake_state.height = 1e7
    agent = Agent(
        fake_state,
        default_interceptor(),
        GuidanceParams(law=law, nav_constant=nav_constant),
    )
    agent.pos = Vector(5_000, 5_000)
    agent.vel = Vector(120, 0)
    agent.reorient()
    fake_state.targets = [target]

    los_rates, headings, ranges = [], [], []
    for _ in range(steps):
        _, range_m, _, rate = engagement_geometry(agent, target)
        los_rates.append(abs(rate))
        headings.append(agent.forward_vec.angle())
        ranges.append(range_m)
        if range_m < agent.hit_radius_m + target.hit_radius_m:
            break
        agent.step(dt)
        target.step(dt)

    return agent, los_rates, headings, ranges


# --- geometry -----------------------------------------------------------


def test_engagement_geometry_on_a_head_on_closing_pair():
    class Stub:
        pass

    agent, target = Stub(), Stub()
    agent.pos, agent.vel = Vector(0, 0), Vector(100, 0)
    target.pos, target.vel = Vector(1000, 0), Vector(-50, 0)

    los, range_m, closing, los_rate = engagement_geometry(agent, target)
    assert range_m == pytest.approx(1000.0)
    assert closing == pytest.approx(150.0)  # 100 + 50
    assert los_rate == pytest.approx(0.0)  # pure head-on: bearing is fixed


def test_crossing_geometry_has_a_nonzero_los_rate():
    class Stub:
        pass

    agent, target = Stub(), Stub()
    agent.pos, agent.vel = Vector(0, 0), Vector(100, 0)
    target.pos, target.vel = Vector(100, 0), Vector(0, 50)

    _, _, closing, los_rate = engagement_geometry(agent, target)
    assert closing == pytest.approx(100.0)
    assert los_rate == pytest.approx(0.5)


def test_los_normal_is_perpendicular_and_unit():
    los = Vector(30, 40)
    normal = los_normal(los, los.magnitude())
    assert normal.magnitude() == pytest.approx(1.0)
    assert normal.dot(los) == pytest.approx(0.0)


def test_zero_range_geometry_is_safe():
    class Stub:
        pass

    a, t = Stub(), Stub()
    a.pos = t.pos = Vector(1, 1)
    a.vel = t.vel = Vector(0, 0)
    _, range_m, closing, rate = engagement_geometry(a, t)
    assert (range_m, closing, rate) == (0.0, 0.0, 0.0)


# --- law behaviour ------------------------------------------------------


def test_pn_drives_los_rate_toward_zero(fake_state):
    """The defining property: a collision course is lambda_dot == 0."""
    target = ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    _, los_rates, _, ranges = fly(fake_state, "pn", target)

    assert ranges[-1] < 3.0, "PN failed to intercept"
    assert los_rates[0] > 0.01
    assert los_rates[-1] < los_rates[0] / 10


def test_pure_pursuit_lets_los_rate_grow(fake_state):
    """
    The contrast case. Pursuit always points at the target's current
    position, so it converts into a tail chase and the sight line spins
    faster as range closes.
    """
    target = ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    _, los_rates, _, ranges = fly(fake_state, "pursuit", target)

    assert ranges[-1] < 3.0
    assert los_rates[-1] > los_rates[0]


def test_pn_flies_a_straight_terminal_course(fake_state):
    """Zero LOS rate means the interceptor stops turning near the end."""
    target = ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    _, _, headings, _ = fly(fake_state, "pn", target)

    pn_drift = abs(headings[-1] - headings[-31])  # last half second

    target2 = ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    _, _, pursuit_headings, _ = fly(fake_state, "pursuit", target2)
    pursuit_drift = abs(pursuit_headings[-1] - pursuit_headings[-31])

    assert pn_drift < 0.01
    assert pn_drift < pursuit_drift / 10


def test_pn_spends_less_control_effort_than_pursuit(fake_state):
    """PN is near-optimal in control effort; that is the point of it."""
    pn_agent, _, _, _ = fly(
        fake_state, "pn", ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    )
    pursuit_agent, _, _, _ = fly(
        fake_state,
        "pursuit",
        ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60)),
    )
    assert pn_agent.delta_v_mps < pursuit_agent.delta_v_mps


def test_apn_reduces_to_pn_against_a_non_manoeuvring_target(fake_state):
    """
    The augmentation term is proportional to target acceleration, so it must
    vanish exactly when the target is not accelerating.
    """
    pn_agent, pn_rates, _, _ = fly(
        fake_state, "pn", ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    )
    apn_agent, apn_rates, _, _ = fly(
        fake_state, "apn", ConstantVelocityTarget(Vector(6_000, 5_000), Vector(0, 60))
    )
    assert apn_agent.delta_v_mps == pytest.approx(pn_agent.delta_v_mps)
    assert apn_rates == pn_rates


def test_apn_diverges_from_pn_when_the_target_accelerates(fake_state):
    pn = ProportionalNavigation(nav_constant=4.0)
    apn = AugmentedProportionalNavigation(nav_constant=4.0)

    fake_state.width = fake_state.height = 1e7
    agent = Agent(fake_state, default_interceptor(), GuidanceParams(law="pn"))
    agent.pos, agent.vel = Vector(0, 0), Vector(120, 0)
    agent.reorient()

    target = ConstantVelocityTarget(Vector(1000, 200), Vector(0, 60))
    target.acc = Vector(0, 80)  # pulling away laterally

    assert apn.lateral_accel(agent, target).magnitude() != pytest.approx(
        pn.lateral_accel(agent, target).magnitude()
    )


def test_pn_falls_back_to_pursuit_when_range_is_opening(fake_state):
    """
    With negative closing velocity the PN command changes sign and steers
    away, so the law must gate on closure.
    """
    fake_state.width = fake_state.height = 1e7
    agent = Agent(fake_state, default_interceptor(), GuidanceParams(law="pn"))
    agent.pos, agent.vel = Vector(0, 0), Vector(-120, 0)  # flying away
    agent.reorient()
    target = ConstantVelocityTarget(Vector(1000, 0), Vector(200, 0))

    _, _, closing, _ = engagement_geometry(agent, target)
    assert closing < 0

    pn_force = ProportionalNavigation().command(agent, target, dt=1 / 60)
    pursuit_force = PurePursuit().command(agent, target, dt=1 / 60)
    assert pn_force.x == pytest.approx(pursuit_force.x)
    assert pn_force.y == pytest.approx(pursuit_force.y)


# --- allocation and wiring ----------------------------------------------


def test_commanded_force_never_exceeds_the_airframe_limit(fake_state):
    fake_state.width = fake_state.height = 1e7
    for law in sorted(LAWS):
        agent = Agent(fake_state, default_interceptor(), GuidanceParams(law=law))
        agent.pos, agent.vel = Vector(0, 0), Vector(5, 0)  # far below max speed
        agent.reorient()
        target = ConstantVelocityTarget(Vector(50, 40), Vector(-80, 0))
        force = agent.guidance.command(agent, target, dt=1 / 60)
        assert force.magnitude() <= agent.max_force * (1 + 1e-9), law


def test_allocation_spends_spare_budget_on_closing_to_top_speed(fake_state):
    """A head-on geometry needs no turn, so all the budget goes axial."""
    fake_state.width = fake_state.height = 1e7
    agent = Agent(fake_state, default_interceptor(), GuidanceParams(law="pn"))
    agent.pos, agent.vel = Vector(0, 0), Vector(40, 0)
    agent.reorient()
    target = ConstantVelocityTarget(Vector(1000, 0), Vector(-50, 0))

    force = agent.guidance.command(agent, target, dt=1 / 60)
    assert force.dot(agent.forward_vec) > 0
    assert force.magnitude() == pytest.approx(agent.max_force)


def test_make_guidance_builds_each_law_and_passes_the_nav_constant():
    for name in LAWS:
        law = make_guidance(GuidanceParams(law=name, nav_constant=3.5))
        assert law.name == name
        if isinstance(law, ProportionalNavigation):
            assert law.nav_constant == 3.5


def test_unknown_guidance_law_is_rejected():
    with pytest.raises(ValueError, match="unknown guidance law"):
        make_guidance(GuidanceParams(law="telepathy"))


def test_nav_constant_changes_the_command(fake_state):
    fake_state.width = fake_state.height = 1e7
    agent = Agent(fake_state, default_interceptor(), GuidanceParams(law="pn"))
    agent.pos, agent.vel = Vector(0, 0), Vector(120, 0)
    agent.reorient()
    target = ConstantVelocityTarget(Vector(1000, 100), Vector(0, 60))

    low = ProportionalNavigation(nav_constant=3.0).lateral_accel(agent, target)
    high = ProportionalNavigation(nav_constant=5.0).lateral_accel(agent, target)
    assert high.magnitude() > low.magnitude()


def test_agent_diagnostics_are_empty_with_no_targets(fake_state):
    agent = Agent(fake_state, default_interceptor(), GuidanceParams(law="pn"))
    fake_state.targets = []
    assert agent.diagnostics() == {}
