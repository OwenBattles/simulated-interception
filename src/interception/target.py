import math

from .actor import Actor
from .constants import (
    TARGET_WANDER_CIRCLE_DIST_M,
    TARGET_WANDER_CIRCLE_RADIUS_M,
    TARGET_WANDER_MAX_RAD,
    TARGET_WANDER_SIGMA_RAD_PER_SQRT_S,
)
from .params import default_target
from .vector import Vector


class Target(Actor):
    """
    Evader flying a Reynolds wander: steer toward a point on a circle
    projected ahead of the nose, where that point drifts around the circle
    as a random walk.

    The walk increment scales with ``sqrt(dt)`` rather than ``dt`` so the
    path statistics are the same at any timestep -- a plain ``* dt`` would
    make the evader smoother simply by running the sim faster.

    This target is non-adversarial: it does not react to being chased.
    """

    def __init__(self, state_ref, params=None):
        super().__init__(state_ref, params or default_target())
        self.wander_angle = 0.0

    def steering_force_at(self, dt):
        avoidance = self.obstacle_avoidance_force()
        if avoidance.magnitude_squared() > 0.0:
            return avoidance
        return self.wander_force(dt)

    def wander_force(self, dt):
        rng = self.state_ref.rng
        self.wander_angle += rng.gauss(0.0, 1.0) * (
            TARGET_WANDER_SIGMA_RAD_PER_SQRT_S * math.sqrt(dt)
        )
        self.wander_angle = max(
            -TARGET_WANDER_MAX_RAD, min(TARGET_WANDER_MAX_RAD, self.wander_angle)
        )

        circle_centre = self.pos + self.forward_vec * TARGET_WANDER_CIRCLE_DIST_M
        offset = Vector.from_polar(
            TARGET_WANDER_CIRCLE_RADIUS_M,
            self.forward_vec.angle() + self.wander_angle,
        )
        wander_point = circle_centre + offset

        desired_vel = (wander_point - self.pos).set_magnitude(self.max_speed)
        return self.force_to_reach(desired_vel, dt)
