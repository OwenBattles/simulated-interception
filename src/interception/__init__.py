"""
2D interception simulator: guidance, obstacle avoidance, and engagement
metrics for a counter-UAS scenario.

The core (state, actors, guidance, collision) is pure Python with no
rendering dependency. Rendering lives in :mod:`interception.view` and is
imported only when a window is actually requested.
"""

from .agent import Agent
from .fleet import Fleet
from .obstacle import Obstacle
from .simulation import EpisodeEnd, Simulation, SimulationConfig, run_headless
from .state import State
from .target import Target
from .vector import Vector

__version__ = "0.2.0"

__all__ = [
    "Agent",
    "EpisodeEnd",
    "Fleet",
    "Obstacle",
    "Simulation",
    "SimulationConfig",
    "State",
    "Target",
    "Vector",
    "run_headless",
]
