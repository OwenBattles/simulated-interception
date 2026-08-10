import math
import random

from .collision import swept_hit
from .fleet import Fleet
from .obstacle import Obstacle
from .params import ScenarioParams
from .target import Target


class State:
    """
    The world: extent in metres, a seeded RNG, and every entity in it.

    Pygame-free by construction, so the whole model can run headless in CI
    without an SDL surface.
    """

    def __init__(self, seed, scenario=None):
        self.scenario = scenario or ScenarioParams()
        self.width = self.scenario.world_width_m
        self.height = self.scenario.world_height_m

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
        cfg = self.scenario
        n_obstacles = self.rng.randint(cfg.min_obstacles, cfg.max_obstacles)
        self.obstacles = [Obstacle(self) for _ in range(n_obstacles)]
        self.targets = [Target(self, cfg.target) for _ in range(cfg.num_targets)]
        self.fleet = Fleet(cfg.num_agents, self, cfg.interceptor, cfg.guidance)
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
        self._resolve_intercepts()

    def _resolve_intercepts(self):
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
