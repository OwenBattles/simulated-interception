import pytest

from interception.constants import SIM_DT
from interception.state import State


def snapshot(state):
    return [(a.pos.x, a.pos.y, a.vel.x, a.vel.y) for a in state.actors] + [
        (o.pos.x, o.pos.y, o.radius_m) for o in state.obstacles
    ]


def test_same_seed_builds_the_same_world():
    assert snapshot(State(seed=7)) == snapshot(State(seed=7))


def test_different_seeds_build_different_worlds():
    assert snapshot(State(seed=7)) != snapshot(State(seed=8))


def test_reset_without_a_seed_replays_the_same_world():
    """
    Regression: ``reset`` used to call ``rng.seed(None)``, which reseeds
    from OS entropy and quietly broke the reproducibility guarantee.
    """
    state = State(seed=11)
    original = snapshot(state)

    for _ in range(50):
        state.update(SIM_DT)
    state.reset()

    assert snapshot(state) == original


def test_reset_with_a_seed_switches_worlds_and_sticks():
    state = State(seed=11)
    state.reset(12)
    assert snapshot(state) == snapshot(State(seed=12))

    state.update(SIM_DT)
    state.reset()  # replays 12, not 11
    assert snapshot(state) == snapshot(State(seed=12))


def test_simultaneous_intercepts_all_resolve(place):
    """
    Regression: kills were applied by removing from ``targets``/``actors``
    while :meth:`State.update` iterated ``actors``, which skipped entries.
    Two targets destroyed on the same tick must both be removed, and every
    surviving actor must still have advanced.
    """
    state = State(seed=3, num_agents=1, num_targets=3)
    agent = state.agents[0]
    doomed = state.targets[:2]
    survivor = state.targets[2]
    for target in doomed:
        target.pos = agent.pos.copy()

    before = {id(a): a.pos.copy() for a in state.actors}
    state.update(SIM_DT)

    assert state.intercepts == 2
    assert state.targets == [survivor]
    assert survivor in state.actors
    for target in doomed:
        assert target not in state.actors

    for actor in state.actors:
        assert actor.pos != before[id(actor)], "an actor was skipped during the step"


def test_min_miss_distance_is_tracked():
    state = State(seed=3)
    assert state.min_miss_distance_m == float("inf")
    for _ in range(200):
        state.update(SIM_DT)
    assert state.min_miss_distance_m < float("inf")


def test_obstacles_are_not_actors():
    """Static geometry must stay out of the step loop."""
    state = State(seed=5)
    assert state.obstacles
    for obstacle in state.obstacles:
        assert obstacle not in state.actors
        assert not hasattr(obstacle, "vel")


def test_obstacles_are_spawned_inside_the_world():
    state = State(seed=5)
    for o in state.obstacles:
        assert o.radius_m <= o.pos.x <= state.width - o.radius_m
        assert o.radius_m <= o.pos.y <= state.height - o.radius_m


def test_actors_stay_inside_the_world():
    state = State(seed=5)
    for _ in range(600):
        state.update(SIM_DT)
        for actor in state.actors:
            assert 0.0 <= actor.pos.x <= state.width
            assert 0.0 <= actor.pos.y <= state.height
