"""
Differential tests: the Python reference engine against the C++ engine.

This is what validates the port. Both implementations consume the same
seeded PCG32 stream in the same order and apply the same arithmetic, so
their observations must agree exactly -- not approximately.

Two real bugs were caught only by this comparison and would have been
invisible to inspection or to either suite alone:

- C++ leaves function argument evaluation order unspecified, so building a
  vector from two rng calls could consume the stream backwards.
- CPython's math.hypot is not libm's, and the ~5e-13 disagreement compounded
  until it flipped which target was nearest.

These tests exist for the length of the transition. Once the Python engine
is removed, the golden fixtures in test_golden.py take over the same job.
"""

import pytest

from interception import _core
from interception.params import GuidanceParams as PyGuidance
from interception.params import ScenarioParams as PyScenario
from interception.simulation import Simulation as PySimulation
from interception.simulation import SimulationConfig as PyConfig
from interception.simulation import run_headless as py_run_headless

LAWS = ("pursuit", "lead", "pn", "apn")
SEEDS = (0, 1, 2, 3, 7)
FLEETS = ((1, 1), (2, 3))
MAX_STEPS = 4000


def python_episode(seed, law, agents, targets):
    scenario = PyScenario(
        num_agents=agents,
        num_targets=targets,
        guidance=PyGuidance(law=law),
    )
    return py_run_headless(
        seed=seed, max_steps=MAX_STEPS, scenario=scenario
    ).observation()


def cpp_episode(seed, law, agents, targets):
    scenario = _core.ScenarioParams(
        num_agents=agents,
        num_targets=targets,
        guidance=_core.GuidanceParams(law=law),
    )
    return _core.run_headless(
        seed=seed, max_steps=MAX_STEPS, scenario=scenario
    ).observation()


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("seed", SEEDS)
def test_engines_agree_one_on_one(seed, law):
    assert cpp_episode(seed, law, 1, 1) == python_episode(seed, law, 1, 1)


@pytest.mark.parametrize("agents,targets", FLEETS)
def test_engines_agree_on_fleet_engagements(agents, targets):
    for seed in (0, 5):
        assert cpp_episode(seed, "pn", agents, targets) == python_episode(
            seed, "pn", agents, targets
        )


def test_engines_agree_on_world_generation():
    """Entity layout must match before anything has moved."""
    py_state = py_run_headless(seed=11, max_steps=1).state
    cpp_state = _core.run_headless(seed=11, max_steps=1).state

    assert len(py_state.obstacles) == len(cpp_state.obstacles)
    for py_obstacle, cpp_obstacle in zip(py_state.obstacles, cpp_state.obstacles):
        assert py_obstacle.pos.x == pytest.approx(cpp_obstacle.pos.x, abs=1e-12)
        assert py_obstacle.pos.y == pytest.approx(cpp_obstacle.pos.y, abs=1e-12)
        assert py_obstacle.radius_m == pytest.approx(cpp_obstacle.radius_m, abs=1e-12)


def test_engines_agree_step_by_step():
    """
    Positions must track for the whole episode, not just at the end.

    An end-of-episode comparison can hide a divergence that happens to
    reconverge, so this walks the trajectory. It reads state rather than
    telemetry: the recorders round their frames, and rounding would mask
    exactly the small drift this test exists to catch.
    """
    py_sim = PySimulation(PyConfig(seed=2, max_steps=600))
    cpp_sim = _core.Simulation(_core.SimulationConfig(seed=2, max_steps=600))

    while not py_sim.done:
        py_sim.step()
        cpp_sim.step()

        assert py_sim.steps == cpp_sim.steps
        assert len(py_sim.state.targets) == len(cpp_sim.state.targets)

        py_actors = list(py_sim.state.agents) + list(py_sim.state.targets)
        cpp_actors = list(cpp_sim.state.agents) + list(cpp_sim.state.targets)
        for py_actor, cpp_actor in zip(py_actors, cpp_actors):
            assert py_actor.pos.x == cpp_actor.pos.x
            assert py_actor.pos.y == cpp_actor.pos.y
            assert py_actor.vel.x == cpp_actor.vel.x
            assert py_actor.vel.y == cpp_actor.vel.y

    assert cpp_sim.done


def test_unseeded_cpp_run_is_replayable():
    sim = _core.run_headless(max_steps=2000)
    replay = _core.run_headless(seed=sim.seed, max_steps=2000)
    assert replay.observation() == sim.observation()
