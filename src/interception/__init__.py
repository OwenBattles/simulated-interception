"""
2D interception simulator: guidance, obstacle avoidance, and engagement
metrics for a counter-UAS scenario.

The core (state, actors, guidance, collision) is pure Python with no
rendering dependency. Rendering lives in :mod:`interception.view` and is
imported only when a window is actually requested.
"""

from .agent import Agent
from .fleet import Fleet
from .guidance import (
    AugmentedProportionalNavigation,
    LeadPursuit,
    ProportionalNavigation,
    PurePursuit,
    make_guidance,
)
from .obstacle import Obstacle
from .params import GuidanceParams, ScenarioParams, VehicleParams
from .simulation import EpisodeEnd, Simulation, SimulationConfig, run_headless
from .state import State
from .target import Target
from .telemetry import TelemetryRecorder
from .vector import Vector

__version__ = "0.3.0"

__all__ = [
    "Agent",
    "AugmentedProportionalNavigation",
    "EpisodeEnd",
    "Fleet",
    "GuidanceParams",
    "LeadPursuit",
    "Obstacle",
    "ProportionalNavigation",
    "PurePursuit",
    "ScenarioParams",
    "Simulation",
    "SimulationConfig",
    "State",
    "Target",
    "TelemetryRecorder",
    "Vector",
    "VehicleParams",
    "make_guidance",
]
