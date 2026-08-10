from .actor import Actor
from .guidance import make_guidance
from .params import GuidanceParams, default_interceptor
from .vector import Vector


class Agent(Actor):
    """
    Interceptor. Steering is delegated to a swappable guidance law
    (see :mod:`interception.guidance`); obstacle avoidance pre-empts it
    whenever the probe is blocked.
    """

    def __init__(self, state_ref, params=None, guidance_params=None):
        super().__init__(state_ref, params or default_interceptor())
        self.guidance = make_guidance(guidance_params or GuidanceParams())

    def steering_force_at(self, dt):
        avoidance = self.obstacle_avoidance_force()
        if avoidance.magnitude_squared() > 0.0:
            return avoidance

        target = self.current_target()
        if target is None:
            return Vector()  # nothing left to chase: coast
        return self.guidance.command(self, target, dt)

    def current_target(self):
        """
        Nearest surviving target, or None once the field is clear.

        Each interceptor chooses independently, so several can converge on
        the same target. Fleet-level assignment is a separate concern.
        """
        if not self.state_ref.targets:
            return None
        return min(self.state_ref.targets, key=lambda t: self.pos.dist_to(t.pos))

    def diagnostics(self):
        """Per-step guidance telemetry, or an empty dict with no target."""
        target = self.current_target()
        if target is None:
            return {}
        return self.guidance.diagnostics(self, target)
