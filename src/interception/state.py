import math
import random

from .collision import swept_hit
from .constants import (
    MAX_OBSTACLE_COUNT,
    MIN_OBSTACLE_COUNT,
    WORLD_HEIGHT_M,
    WORLD_WIDTH_M,
)
from .fleet import Fleet
from .obstacle import Obstacle
from .target import Target


class State:
    """
    The world: extent in metres, a seeded RNG, and every entity in it.

    Pygame-free by construction, so the whole model can run headless in CI
    without an SDL surface.
    """

    def __init__(
        self,
        seed,
        width=None,
        height=None,
        num_agents=1,
        num_targets=1,
    ):
        self.width = WORLD_WIDTH_M if width is None else float(width)
        self.height = WORLD_HEIGHT_M if height is None else float(height)
        self.num_agents = num_agents
        self.num_targets = num_targets

        # A concrete seed is required. Callers that want an arbitrary world
        # draw a seed first and record it, so every run stays reproducible.
        self.seed = int(seed)
        self.rng = random.Random(self.seed)

        self.obstacles = []
        self.targets = []
        self.agents = []
        self.actors = []
        self.intercepts = 0
        self.min_miss_distance_m = math.inf

        self._build_world()

    def _build_world(self):
        n_obstacles = self.rng.randint(MIN_OBSTACLE_COUNT, MAX_OBSTACLE_COUNT)
        self.obstacles = [Obstacle(self) for _ in range(n_obstacles)]
        self.targets = [Target(self) for _ in range(self.num_targets)]
        self.fleet = Fleet(self.num_agents, self)
        self.agents = list(self.fleet.agents)
        # Obstacles are static, so they are not actors and never step.
        self.actors = self.agents + self.targets
        self.intercepts = 0
        self.min_miss_distance_m = math.inf

    def reset(self, seed=None):
        """
        Rebuild the world from a known seed.

        Passing ``None`` reuses the seed this world was built with rather
        than reseeding from OS entropy, so ``reset()`` is repeatable.
        """
        if seed is not None:
            self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self._build_world()

    def update(self, dt):
        for actor in self.actors:
            actor.step(dt)
        self._resolve_intercepts(dt)

    def _resolve_intercepts(self, dt):
        """
        Swept-sphere test over every agent/target pair for this step.

        Removals are deferred until after the scan: mutating ``targets`` or
        ``actors`` mid-iteration silently skips entries.
        """
        doomed = {}
        for agent in self.agents:
            for target in self.targets:
                combined_radius = agent.hit_radius_m + target.hit_radius_m
                hit, miss = swept_hit(
                    agent.prev_pos,
                    agent.pos,
                    target.prev_pos,
                    target.pos,
                    combined_radius,
                )
                if miss < self.min_miss_distance_m:
                    self.min_miss_distance_m = miss
                if hit:
                    doomed[id(target)] = target

        if not doomed:
            return

        self.targets = [t for t in self.targets if id(t) not in doomed]
        self.actors = [a for a in self.actors if id(a) not in doomed]
        self.intercepts += len(doomed)
