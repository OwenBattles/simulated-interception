from .actor import Actor
from .constants import (
    AGENT_HIT_RADIUS_M,
    AGENT_MASS_KG,
    AGENT_MAX_FORCE_N,
    AGENT_MAX_SPEED_MPS,
    AGENT_PROBE_LOOKAHEAD_S,
    PROBE_RADIUS_M,
)
from .sensor import Probe
from .vector import Vector


class Agent(Actor):
    """
    Interceptor running lead pursuit against the nearest target.

    Guidance is deliberately simple at this stage: aim at where the target
    will be after a straight-line time-to-go, and fly there at full speed.
    Obstacle avoidance pre-empts pursuit whenever the probe is blocked.
    """

    def __init__(self, state_ref):
        super().__init__(
            state_ref,
            mass=AGENT_MASS_KG,
            max_speed=AGENT_MAX_SPEED_MPS,
            max_force=AGENT_MAX_FORCE_N,
            hit_radius_m=AGENT_HIT_RADIUS_M,
            probe=Probe(AGENT_PROBE_LOOKAHEAD_S, PROBE_RADIUS_M),
        )

    def steering_force_at(self, dt):
        avoidance = self.obstacle_avoidance_force()
        if avoidance.magnitude_squared() > 0.0:
            return avoidance
        return self.pursuit_force(dt)

    def current_target(self):
        """Nearest surviving target, or None once the field is clear."""
        if not self.state_ref.targets:
            return None
        return min(self.state_ref.targets, key=lambda t: self.pos.dist_to(t.pos))

    def predicted_intercept_point(self, target):
        """
        Lead point from a first-order time-to-go estimate.

        ``t_go = range / closing speed`` is approximated with the
        interceptor's own top speed, which is exact for a head-on
        non-manoeuvring target and degrades in a crossing geometry.
        Replacing this with proportional navigation is the next step.
        """
        range_m = self.pos.dist_to(target.pos)
        time_to_go_s = range_m / self.max_speed
        return target.pos + target.vel * time_to_go_s

    def pursuit_force(self, dt):
        target = self.current_target()
        if target is None:
            return Vector()  # nothing left to chase: coast
        aim_point = self.predicted_intercept_point(target)
        desired_vel = (aim_point - self.pos).set_magnitude(self.max_speed)
        return self.force_to_reach(desired_vel, dt)
