"""
Golden-fixture regression against the original Python engine.

The pure-Python engine was the specification this project started from. It
has been removed now that C++ is the single implementation, but its
behaviour is preserved here: `fixtures/golden.json` was recorded from it at
the commit where the differential suite showed the two engines agreeing
exactly, on 48 episodes spanning four guidance laws, six seeds and two
fleet configurations, plus a sampled trajectory.

So this is not a snapshot of "whatever the code did when I wrote the test".
It is a check that the C++ engine still reproduces an implementation that
was independently written and independently verified.

Regenerating these fixtures from the current engine would defeat the point.
If one fails, the engine changed behaviour -- decide whether that was
intended before touching this file.
"""

import json
import pathlib

import pytest

from interception import _core

GOLDEN = json.loads((pathlib.Path(__file__).parent / "fixtures/golden.json").read_text())


def run(seed, law, agents, targets, max_steps):
    scenario = _core.ScenarioParams(
        num_agents=agents,
        num_targets=targets,
        guidance=_core.GuidanceParams(law=law),
    )
    return _core.run_headless(seed=seed, max_steps=max_steps, scenario=scenario)


@pytest.mark.parametrize(
    "case",
    GOLDEN["episodes"],
    ids=lambda c: f"{c['law']}-seed{c['seed']}-{c['agents']}v{c['targets']}",
)
def test_episode_matches_the_reference_engine(case):
    sim = run(
        case["seed"], case["law"], case["agents"], case["targets"], GOLDEN["max_steps"]
    )
    assert sim.observation() == case["observation"]


def test_trajectory_matches_the_reference_engine():
    """
    Sampled positions along the flight, not just the final observation.

    An end-of-episode check can pass while the path drifts and reconverges.
    """
    spec = GOLDEN["trajectory"]
    sim = _core.Simulation(
        _core.SimulationConfig(seed=spec["seed"], max_steps=spec["max_steps"])
    )

    samples = iter(spec["samples"])
    expected = next(samples, None)
    while not sim.done and expected is not None:
        sim.step()
        if sim.steps != expected["step"]:
            continue

        # Flattened because pytest.approx does not descend into nested lists.
        actual = [c for a in sim.state.agents for c in (a.pos.x, a.pos.y)]
        actual += [c for t in sim.state.targets for c in (t.pos.x, t.pos.y)]
        want = [c for pair in expected["agents"] + expected["targets"] for c in pair]
        assert actual == pytest.approx(want, abs=1e-9)
        expected = next(samples, None)

    assert expected is None, "engine ended the episode before the fixtures ran out"
