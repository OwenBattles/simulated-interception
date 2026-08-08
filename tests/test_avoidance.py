import pytest

from interception.agent import Agent
from interception.obstacle import Obstacle
from interception.vector import Vector


def test_two_obstacles_in_probe_range_does_not_crash(fake_state, place):
    """
    Regression: comparing distances to obstacles used to pass the Obstacle
    object where a Vector was expected. Short-circuit ``or`` hid it for the
    first obstacle, so the crash only fired once a second obstacle entered
    probe range.
    """
    agent = place(Agent(fake_state), pos=(500, 500), vel=(120, 0))
    fake_state.obstacles = [
        Obstacle(fake_state, pos=Vector(596, 500), radius_m=60.0),
        Obstacle(fake_state, pos=Vector(600, 520), radius_m=60.0),
    ]

    force = agent.obstacle_avoidance_force()

    assert force.magnitude() > 0.0


def test_nearest_obstacle_wins(fake_state, place):
    """With two threats in range, avoidance must respond to the closer one."""
    agent = place(Agent(fake_state), pos=(500, 500), vel=(120, 0))
    near = Obstacle(fake_state, pos=Vector(580, 470), radius_m=40.0)
    far = Obstacle(fake_state, pos=Vector(620, 530), radius_m=40.0)

    fake_state.obstacles = [near, far]
    force_near_first = agent.obstacle_avoidance_force()
    fake_state.obstacles = [far, near]
    force_far_first = agent.obstacle_avoidance_force()

    # Selection must not depend on iteration order.
    assert force_near_first.x == pytest.approx(force_far_first.x)
    assert force_near_first.y == pytest.approx(force_far_first.y)


def test_clear_path_produces_no_avoidance_force(fake_state, place):
    agent = place(Agent(fake_state), pos=(500, 500), vel=(120, 0))
    fake_state.obstacles = [Obstacle(fake_state, pos=Vector(500, 50), radius_m=30.0)]
    assert agent.obstacle_avoidance_force() == Vector(0, 0)


def test_avoidance_steers_away_from_the_obstacle_side(fake_state, place):
    """An obstacle off the right shoulder must produce a leftward command."""
    agent = place(Agent(fake_state), pos=(500, 500), vel=(120, 0))
    # forward = +x, side = perpendicular = +y ("right" in screen space)
    obstacle = Obstacle(fake_state, pos=Vector(590, 520), radius_m=50.0)
    fake_state.obstacles = [obstacle]

    force = agent.obstacle_avoidance_force()

    assert agent.side_vec.dot(obstacle.pos - agent.pos) > 0
    assert agent.side_vec.dot(force) < 0  # turn the other way
    assert agent.forward_vec.dot(force) < 0  # and bleed some speed


def test_probe_standoff_scales_with_speed(fake_state, place):
    """Constant reaction time, not constant reaction distance."""
    slow = place(Agent(fake_state), pos=(500, 500), vel=(30, 0))
    fast = place(Agent(fake_state), pos=(500, 500), vel=(120, 0))

    slow.obstacle_avoidance_force()
    fast.obstacle_avoidance_force()

    slow_standoff = slow.probe.pos.dist_to(slow.pos)
    fast_standoff = fast.probe.pos.dist_to(fast.pos)
    assert fast_standoff == pytest.approx(4 * slow_standoff)
