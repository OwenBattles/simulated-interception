import math
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .constants import DEFAULT_HEADLESS_MAX_STEPS, SIM_DT
from .params import ScenarioParams
from .state import State
from .telemetry import TelemetryRecorder

SEED_SPACE = 2**32


class EpisodeEnd(str, Enum):
    """Why the episode stopped."""

    NONE = "none"
    SUCCESS = "success"
    TIMEOUT = "timeout"


@dataclass
class SimulationConfig:
    """Everything needed to reproduce an episode exactly."""

    dt: float = SIM_DT
    max_steps: int = 0  # 0 = no cap (interactive); >0 enforces TIMEOUT
    seed: Optional[int] = None  # None = draw one and record it
    scenario: ScenarioParams = field(default_factory=ScenarioParams)
    record_telemetry: bool = False


class Simulation:
    """
    Owns world state and the episode lifecycle: reset -> step until terminal.

    Core logic is pygame-free; rendering reads ``state`` from the outside.
    """

    def __init__(self, config: Optional[SimulationConfig] = None):
        self.config = config or SimulationConfig()
        self.dt = self.config.dt
        self.max_steps = self.config.max_steps
        # Resolved once and stored, so an unseeded run is still reportable
        # and replayable after the fact.
        self.seed = self._resolve_seed(self.config.seed)
        self.state = State(seed=self.seed, scenario=self.config.scenario)
        self.telemetry = TelemetryRecorder() if self.config.record_telemetry else None
        self.steps = 0
        self.done = False
        self.end_reason = EpisodeEnd.NONE

    @staticmethod
    def _resolve_seed(seed):
        if seed is not None:
            return int(seed)
        return random.SystemRandom().randrange(SEED_SPACE)

    @property
    def elapsed_s(self):
        return self.steps * self.dt

    @property
    def guidance_law(self):
        return self.config.scenario.guidance.law

    def reset(self, seed: Optional[int] = None) -> None:
        """Rebuild the world. ``seed=None`` replays the current seed."""
        if seed is not None:
            self.seed = int(seed)
        self.state.reset(self.seed)
        if self.telemetry is not None:
            self.telemetry = TelemetryRecorder()
        self.steps = 0
        self.done = False
        self.end_reason = EpisodeEnd.NONE

    def step(self) -> None:
        """Advance one fixed timestep of ``dt`` seconds."""
        if self.done:
            return
        self.state.update(self.dt)
        self.steps += 1
        if self.telemetry is not None:
            self.telemetry.capture(self)
        if not self.state.targets:
            self.done = True
            self.end_reason = EpisodeEnd.SUCCESS
        elif self.max_steps > 0 and self.steps >= self.max_steps:
            self.done = True
            self.end_reason = EpisodeEnd.TIMEOUT

    def run(self):
        """Step until terminal. Requires ``max_steps > 0`` to be bounded."""
        while not self.done:
            self.step()
        return self

    def observation(self) -> dict:
        """Flat, JSON-friendly episode summary for logging and Monte Carlo."""
        miss = self.state.min_miss_distance_m
        return {
            "seed": self.seed,
            "guidance": self.guidance_law,
            "step": self.steps,
            "elapsed_s": round(self.elapsed_s, 4),
            "done": self.done,
            "end_reason": self.end_reason.value,
            "intercepts": self.state.intercepts,
            "min_miss_distance_m": None if math.isinf(miss) else round(miss, 4),
            "delta_v_mps": round(sum(a.delta_v_mps for a in self.state.agents), 3),
            "num_targets": len(self.state.targets),
            "num_agents": len(self.state.agents),
            "num_obstacles": len(self.state.obstacles),
        }


def run_headless(
    seed: Optional[int] = None,
    max_steps: int = DEFAULT_HEADLESS_MAX_STEPS,
    scenario: Optional[ScenarioParams] = None,
    record_telemetry: bool = False,
) -> Simulation:
    """Run until SUCCESS or TIMEOUT; returns the simulation for inspection."""
    cfg = SimulationConfig(
        seed=seed,
        max_steps=max_steps,
        scenario=scenario or ScenarioParams(),
        record_telemetry=record_telemetry,
    )
    return Simulation(cfg).run()
