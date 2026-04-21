from dataclasses import dataclass
from enum import Enum
from typing import Optional

from constants import DEFAULT_HEADLESS_MAX_STEPS, SCREEN_HEIGHT, SCREEN_WIDTH, SIM_DT
from state import State


class EpisodeEnd(str, Enum):
    """Why the episode stopped."""

    NONE = "none"
    SUCCESS = "success"
    TIMEOUT = "timeout"


@dataclass
class SimulationConfig:
    """Configuration for a runnable simulation (headless or visual)."""

    dt: float = SIM_DT
    max_steps: int = 0  # 0 = no step cap (e.g. interactive); >0 enforces TIMEOUT
    seed: Optional[int] = None
    world_width: int = SCREEN_WIDTH
    world_height: int = SCREEN_HEIGHT


class Simulation:
    """
    Owns world state and the episode lifecycle: reset → step until terminal.
    Core logic is pygame-free; rendering reads ``state`` externally.
    """

    def __init__(self, config: Optional[SimulationConfig] = None):
        self.config = config or SimulationConfig()
        self.dt = self.config.dt
        self.max_steps = self.config.max_steps
        self.state = State(
            seed=self.config.seed,
            width=self.config.world_width,
            height=self.config.world_height,
        )
        self.steps = 0
        self.done = False
        self.end_reason = EpisodeEnd.NONE

    def reset(self, seed: Optional[int] = None) -> None:
        """Rebuild the world. ``seed`` defaults to ``config.seed``."""
        effective = self.config.seed if seed is None else seed
        self.state.reset(effective)
        self.steps = 0
        self.done = False
        self.end_reason = EpisodeEnd.NONE

    def step(self) -> None:
        """Advance one integration tick."""
        if self.done:
            return
        self.state.update()
        self.steps += 1
        if not self.state.targets:
            self.done = True
            self.end_reason = EpisodeEnd.SUCCESS
        elif self.max_steps > 0 and self.steps >= self.max_steps:
            self.done = True
            self.end_reason = EpisodeEnd.TIMEOUT

    def observation(self) -> dict:
        """Lightweight snapshot for logging (expand in later phases)."""
        return {
            "step": self.steps,
            "done": self.done,
            "end_reason": self.end_reason.value,
            "num_targets": len(self.state.targets),
            "num_agents": len(self.state.agents),
            "num_obstacles": len(self.state.obstacles),
        }


def run_headless(
    seed: Optional[int] = None,
    max_steps: int = DEFAULT_HEADLESS_MAX_STEPS,
    world_width: int = SCREEN_WIDTH,
    world_height: int = SCREEN_HEIGHT,
) -> Simulation:
    """Run until SUCCESS or TIMEOUT; returns the simulation for inspection."""
    cfg = SimulationConfig(
        seed=seed,
        max_steps=max_steps,
        world_width=world_width,
        world_height=world_height,
    )
    sim = Simulation(cfg)
    while not sim.done:
        sim.step()
    return sim
