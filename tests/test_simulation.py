import sys

import pytest

from interception.simulation import (
    EpisodeEnd,
    Simulation,
    SimulationConfig,
    run_headless,
)


def test_core_imports_without_pygame():
    """
    CI installs the base package only. If any core module grows a pygame
    import, headless runs start needing an SDL surface -- fail loudly here
    rather than mysteriously on a build machine.
    """
    for name in list(sys.modules):
        if name == "pygame" or name.startswith("pygame."):
            pytest.skip("pygame already imported by another test session")

    import interception  # noqa: F401
    import interception.state  # noqa: F401

    assert "pygame" not in sys.modules


def test_headless_episode_terminates():
    sim = run_headless(seed=0, max_steps=5_000)
    assert sim.done
    assert sim.end_reason in (EpisodeEnd.SUCCESS, EpisodeEnd.TIMEOUT)


def test_step_cap_produces_a_timeout():
    sim = run_headless(seed=0, max_steps=5)
    assert sim.end_reason == EpisodeEnd.TIMEOUT
    assert sim.steps == 5
    assert sim.state.targets


def test_success_clears_the_field():
    sim = run_headless(seed=0, max_steps=5_000)
    assert sim.end_reason == EpisodeEnd.SUCCESS
    assert sim.state.targets == []
    assert sim.state.intercepts == 1


def test_elapsed_time_tracks_steps_and_dt():
    sim = run_headless(seed=0, max_steps=100)
    assert sim.elapsed_s == pytest.approx(100 * sim.dt)


def test_same_seed_reproduces_the_episode():
    a = run_headless(seed=42, max_steps=5_000).observation()
    b = run_headless(seed=42, max_steps=5_000).observation()
    assert a == b


def test_unseeded_run_records_a_concrete_replayable_seed():
    """An arbitrary run must still be reproducible after the fact."""
    sim = Simulation(SimulationConfig(seed=None, max_steps=2_000)).run()
    assert isinstance(sim.seed, int)

    replay = run_headless(seed=sim.seed, max_steps=2_000)
    assert replay.observation() == sim.observation()


def test_reset_replays_the_same_episode():
    sim = Simulation(SimulationConfig(seed=9, max_steps=2_000)).run()
    first = sim.observation()
    sim.reset()
    assert sim.steps == 0
    assert not sim.done
    sim.run()
    assert sim.observation() == first


def test_stepping_a_finished_episode_is_a_no_op():
    sim = run_headless(seed=0, max_steps=5)
    steps = sim.steps
    sim.step()
    assert sim.steps == steps


def test_observation_is_json_friendly():
    obs = run_headless(seed=0, max_steps=5_000).observation()
    expected = {
        "seed",
        "step",
        "elapsed_s",
        "done",
        "end_reason",
        "intercepts",
        "min_miss_distance_m",
        "delta_v_mps",
        "num_targets",
        "num_agents",
        "num_obstacles",
    }
    assert set(obs) == expected
    assert isinstance(obs["end_reason"], str)
    assert obs["min_miss_distance_m"] is None or isinstance(
        obs["min_miss_distance_m"], float
    )


def test_multi_agent_multi_target_engagement_runs():
    sim = run_headless(seed=4, max_steps=5_000, num_agents=3, num_targets=2)
    assert len(sim.state.agents) == 3
    assert sim.done
