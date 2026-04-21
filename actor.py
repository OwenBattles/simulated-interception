import random

from vector import Vector
from constants import MAX_X_VEL, MAX_Y_VEL, MAX_X_ACC, MAX_Y_ACC, SCREEN_WIDTH, SCREEN_HEIGHT


class Actor:
    def __init__(self, state_ref):
        self.state_ref = state_ref
        self._rng = getattr(state_ref, "rng", random)
        w = getattr(state_ref, "width", SCREEN_WIDTH)
        h = getattr(state_ref, "height", SCREEN_HEIGHT)
        self.pos = Vector(self._rng.randint(0, w), self._rng.randint(0, h))
        self.vel = Vector(
            self._rng.randint(-MAX_X_VEL, MAX_X_VEL),
            self._rng.randint(-MAX_Y_VEL, MAX_Y_VEL),
        )
        self.acc = Vector(
            self._rng.randint(-MAX_X_ACC, MAX_X_ACC),
            self._rng.randint(-MAX_Y_ACC, MAX_Y_ACC),
        )
        self.mass = 10
        self.length = 0
        self.max_speed = 5
        self.max_force = 4
        self.forward_vec = Vector(1, 0)
        self.side_vec = Vector(0, 1)
        self.probe_distance = 0
        self.probe = None

    def move(self):
        pass

    def draw(self, screen):
        pass

    def reorient(self):
        self.forward_vec = self.vel.set_magnitude(1)
        self.side_vec = self.forward_vec.perpendicular()

    def calculate_obstacle_avoidance(self):
            most_threatening = None
            self.probe.pos = self.pos + self.vel.normalize() * self.probe.radius
            
            for obstacle in self.state_ref.obstacles:
                if self.probe.intersects_obstacle(obstacle):
                    if most_threatening is None or self.pos.dist_to(obstacle) < self.pos.dist_to(most_threatening):
                        most_threatening = obstacle

            if most_threatening:
                dot_product = self.side_vec.dot(most_threatening.pos - self.pos)
                side_steer = -1 if dot_product > 0 else 1
                
                braking_weight = 0.2 # TODO: make this dynamic or a constant
                avoidance_force = self.side_vec * side_steer * self.max_force
                avoidance_force -= self.forward_vec * braking_weight
                
                return avoidance_force
        
            return Vector(0, 0)
