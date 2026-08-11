"""
Tests for the Python-facing surface of the C++ engine.

Engine behaviour is covered by the C++ suite (core/tests) and by the golden
fixtures. What is tested here is the boundary: that the bindings expose what
the CLI, the pygame view, and the figure scripts actually use, and that the
borrow semantics behave as documented.
"""

import json

import pytest

from interception import _core, telemetry
from interception.cli import build_parser, main, scenario_from, summarise


def test_importing_the_package_does_not_pull_in_pygame():
    """
    CI installs the base package with no pygame at all. If the package root
    or the CLI grows a rendering import, headless runs start needing an SDL
    surface -- fail loudly here rather than mysteriously on a build machine.
    """
    import subprocess
    import sys

    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys, interception, interception.cli;"
            " assert 'pygame' not in sys.modules",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_module_constants_are_exposed():
    assert _core.SIM_DT == pytest.approx(1 / 60)
    assert set(_core.GUIDANCE_LAWS) == {"pursuit", "lead", "pn", "apn"}
    assert _core.WORLD_WIDTH_M == 1500.0
    assert _core.WORLD_HEIGHT_M == 1000.0


def test_vehicle_params_expose_derived_quantities():
    params = _core.default_interceptor()
    assert params.max_accel_mps2 == pytest.approx(params.max_force_n / params.mass_kg)
    assert params.turn_radius_m == pytest.approx(
        params.max_speed_mps**2 / params.max_accel_mps2
    )
    assert params.max_accel_g == pytest.approx(params.max_accel_mps2 / 9.80665)


def test_guidance_params_round_trip_through_strings():
    for law in _core.GUIDANCE_LAWS:
        assert _core.GuidanceParams(law=law).law == law


def test_unknown_guidance_law_is_rejected():
    with pytest.raises(ValueError):
        _core.GuidanceParams(law="telepathy")


def test_scenario_params_accept_partial_overrides():
    scenario = _core.ScenarioParams(num_agents=3, num_targets=2)
    assert scenario.num_agents == 3
    assert scenario.num_targets == 2
    # Untouched fields keep the engine defaults.
    assert scenario.world_width_m == 1500.0
    assert scenario.interceptor.max_speed_mps == 120.0


def test_state_exposes_what_the_renderer_needs():
    sim = _core.run_headless(seed=0, max_steps=50)
    state = sim.state
    assert state.width == 1500.0 and state.height == 1000.0
    assert state.obstacles

    agent = state.agents[0]
    for attr in ("pos", "vel", "forward_vec", "side_vec", "probe"):
        assert hasattr(agent, attr)
    assert agent.probe.radius_m > 0
    assert agent.pos.pair() == (agent.pos.x, agent.pos.y)
    assert state.obstacles[0].radius_m > 0


def test_actor_views_reflect_motion_when_re_read():
    """
    Handles borrow from the State, so callers re-read them each step rather
    than caching. Re-reading must show the new position.
    """
    sim = _core.Simulation(_core.SimulationConfig(seed=0, max_steps=100))
    before = sim.state.agents[0].pos.pair()
    for _ in range(20):
        sim.step()
    assert sim.state.agents[0].pos.pair() != before


def test_same_seed_reproduces_the_episode():
    a = _core.run_headless(seed=42, max_steps=5000).observation()
    b = _core.run_headless(seed=42, max_steps=5000).observation()
    assert a == b


def test_unseeded_run_records_a_replayable_seed():
    sim = _core.run_headless(max_steps=2000)
    assert isinstance(sim.seed, int)
    assert _core.run_headless(seed=sim.seed, max_steps=2000).observation() == (
        sim.observation()
    )


def test_reset_replays_the_same_episode():
    sim = _core.Simulation(_core.SimulationConfig(seed=9, max_steps=2000))
    sim.run()
    first = sim.observation()
    sim.reset()
    assert sim.steps == 0 and not sim.done
    sim.run()
    assert sim.observation() == first


def test_step_cap_produces_a_timeout():
    sim = _core.run_headless(seed=0, max_steps=5)
    assert sim.end_reason == "timeout"
    assert sim.steps == 5


def test_observation_shape():
    obs = _core.run_headless(seed=0, max_steps=5000).observation()
    assert set(obs) == {
        "seed",
        "guidance",
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
    json.dumps(obs)  # must be serialisable as-is


def test_telemetry_is_opt_in_and_non_perturbing():
    plain = _core.run_headless(seed=0, max_steps=200)
    assert plain.telemetry is None
    assert telemetry.frames(plain) == []

    recorded = _core.run_headless(seed=0, max_steps=200, record_telemetry=True)
    assert len(telemetry.frames(recorded)) == recorded.steps
    assert recorded.observation() == plain.observation()


def test_telemetry_series_and_json(tmp_path):
    sim = _core.run_headless(seed=0, max_steps=300, record_telemetry=True)
    series = telemetry.series(sim, "los_rate_rad_s")
    assert series
    assert [t for t, _ in series] == sorted(t for t, _ in series)

    path = telemetry.write_json(sim, tmp_path / "run.json")
    payload = json.loads(path.read_text())
    assert payload["summary"] == sim.observation()
    assert len(payload["frames"]) == sim.steps


def test_trace_guidance_separates_the_laws():
    """The comparison harness the figure script draws from."""
    traces = {law: _core.trace_guidance(law=law) for law in _core.GUIDANCE_LAWS}
    for law, trace in traces.items():
        assert trace.intercepted, law
        assert trace.points

    # Against a steady turn, the textbook ordering on control effort.
    assert traces["apn"].delta_v_mps < traces["pn"].delta_v_mps
    assert traces["pn"].delta_v_mps < traces["pursuit"].delta_v_mps
    # And PN nulls the sight-line rotation that pursuit lets run away.
    assert traces["pn"].points[-1].los_rate_rad_s < (
        traces["pursuit"].points[-1].los_rate_rad_s
    )


# --- CLI ---------------------------------------------------------------


def test_cli_scenario_from_args():
    args = build_parser().parse_args(
        ["--agents", "2", "--targets", "3", "--guidance", "apn", "--nav-constant", "3.5"]
    )
    scenario = scenario_from(args)
    assert scenario.num_agents == 2
    assert scenario.num_targets == 3
    assert scenario.guidance.law == "apn"
    assert scenario.guidance.nav_constant == pytest.approx(3.5)


def test_cli_headless_run(capsys):
    main(["--headless", "--seed", "0"])
    out = capsys.readouterr().out
    assert "guidance=pn" in out
    assert "reason=success" in out


def test_cli_json_output(capsys):
    main(["--headless", "--seed", "0", "--json"])
    payload = json.loads(capsys.readouterr().out)
    assert payload["seed"] == 0
    assert payload["end_reason"] == "success"


def test_cli_trials_summary(capsys):
    main(["--trials", "3", "--seed", "0", "--guidance", "lead"])
    out = capsys.readouterr().out
    assert "trials                  3" in out
    assert "guidance                lead" in out


def test_cli_record_writes_telemetry(tmp_path, capsys):
    path = tmp_path / "nested" / "run.json"
    main(["--seed", "0", "--record", str(path)])
    assert path.exists()
    payload = json.loads(path.read_text())
    assert payload["frames"]
    assert "telemetry ->" in capsys.readouterr().out


def test_summarise_handles_an_all_timeout_batch():
    episodes = [
        _core.run_headless(seed=s, max_steps=3).observation() for s in range(3)
    ]
    summary = summarise(episodes)
    assert summary["success_rate"] == 0.0
    assert summary["mean_time_to_intercept_s"] is None
