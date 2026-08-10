"""
Runtime-tunable parameter objects.

``constants`` holds the default *values*; this module holds the *structures*
that carry them through a run. The distinction matters because every
parameter here is meant to become a control in the planned dashboard, and a
module-level global cannot be changed per-run without leaking state between
episodes.

Nothing in the core reads ``constants`` directly except the defaults below.
"""

from dataclasses import dataclass, field

from . import constants as C


@dataclass
class VehicleParams:
    """Airframe limits for one vehicle class."""

    mass_kg: float
    max_speed_mps: float
    max_force_n: float
    hit_radius_m: float
    probe_lookahead_s: float
    probe_radius_m: float = C.PROBE_RADIUS_M

    @property
    def max_accel_mps2(self):
        return self.max_force_n / self.mass_kg

    @property
    def max_accel_g(self):
        return self.max_accel_mps2 / 9.80665

    @property
    def turn_radius_m(self):
        """Minimum turn radius at top speed, v^2 / a."""
        return self.max_speed_mps**2 / self.max_accel_mps2


def default_interceptor():
    return VehicleParams(
        mass_kg=C.AGENT_MASS_KG,
        max_speed_mps=C.AGENT_MAX_SPEED_MPS,
        max_force_n=C.AGENT_MAX_FORCE_N,
        hit_radius_m=C.AGENT_HIT_RADIUS_M,
        probe_lookahead_s=C.AGENT_PROBE_LOOKAHEAD_S,
    )


def default_target():
    return VehicleParams(
        mass_kg=C.TARGET_MASS_KG,
        max_speed_mps=C.TARGET_MAX_SPEED_MPS,
        max_force_n=C.TARGET_MAX_FORCE_N,
        hit_radius_m=C.TARGET_HIT_RADIUS_M,
        probe_lookahead_s=C.TARGET_PROBE_LOOKAHEAD_S,
    )


@dataclass
class GuidanceParams:
    """Which guidance law the interceptors fly, and its tuning."""

    law: str = "pn"
    nav_constant: float = 4.0  # N in a = N * Vc * lambda-dot; 3-5 is typical


@dataclass
class ScenarioParams:
    """Everything about the engagement except the RNG seed."""

    world_width_m: float = C.WORLD_WIDTH_M
    world_height_m: float = C.WORLD_HEIGHT_M
    num_agents: int = 1
    num_targets: int = 1
    min_obstacles: int = C.MIN_OBSTACLE_COUNT
    max_obstacles: int = C.MAX_OBSTACLE_COUNT
    interceptor: VehicleParams = field(default_factory=default_interceptor)
    target: VehicleParams = field(default_factory=default_target)
    guidance: GuidanceParams = field(default_factory=GuidanceParams)
