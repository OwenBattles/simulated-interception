"""
2D interception simulator: guidance, obstacle avoidance, and engagement
metrics for a counter-UAS scenario.

The simulation itself is C++ (see ``core/``), exposed here through the
``_core`` extension. There is one engine; the native CLI, this package, and
the WebAssembly build all run the same sources, so they cannot drift apart.

What remains in Python is everything that is not physics: the command-line
interface, the pygame view, and the figure-generation scripts.
"""

from ._core import (
    DEFAULT_HEADLESS_MAX_STEPS,
    GUIDANCE_LAWS,
    SIM_DT,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
    Actor,
    Agent,
    GuidanceParams,
    Obstacle,
    Probe,
    ScenarioParams,
    Simulation,
    SimulationConfig,
    State,
    Target,
    TelemetryRecorder,
    Vector,
    VehicleParams,
    default_interceptor,
    default_target,
    run_headless,
    trace_guidance,
)

__version__ = "0.4.0"

__all__ = [
    "Actor",
    "Agent",
    "DEFAULT_HEADLESS_MAX_STEPS",
    "GUIDANCE_LAWS",
    "GuidanceParams",
    "Obstacle",
    "Probe",
    "SIM_DT",
    "ScenarioParams",
    "Simulation",
    "SimulationConfig",
    "State",
    "Target",
    "TelemetryRecorder",
    "Vector",
    "VehicleParams",
    "WORLD_HEIGHT_M",
    "WORLD_WIDTH_M",
    "default_interceptor",
    "default_target",
    "run_headless",
    "trace_guidance",
]
