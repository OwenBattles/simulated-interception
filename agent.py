import pygame

from actor import Actor
from circle import Circle   
from vector import Vector
class Agent(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)
        self.length = 20
        self.width = 10
        self.color = (0, 0, 255)
        self.probe_distance = 50 # Distance in front of the agent where the probe is located
        self.probe = Circle(self, self.state_ref, self.probe_distance) # Circle used for collision detection
         

    def move(self):
        desired_direction = self.get_target_future_pos() - self.pos
        pursuit_force = (desired_direction - self.vel)
        avoidance_force = self.calculate_obstacle_avoidance()

        if avoidance_force.magnitude() > 0:
            steering_direction = avoidance_force 
        else:
            steering_direction = pursuit_force

        self.steering_force = steering_direction.truncate(self.max_force) 
        self.acc = self.steering_force / self.mass
        self.vel = (self.vel + self.acc).truncate(self.max_speed)
        self.pos += self.vel

        self.reorient()
        self.attack()

    def get_target_future_pos(self):
        target = self.state_ref.targets[0] if self.state_ref.targets else None # TODO: handle multiple targets
        if not target:
            return self.pos + self.vel # Arbitrary point in front of the agent if no targets exist
        target_future_pos = target.pos + target.vel * (self.pos - target.pos).magnitude() / self.max_speed
        return target_future_pos
    
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

    def draw(self, screen):
        tip = self.pos + self.forward_vec * self.length
        left_rear = self.pos - self.forward_vec * self.length + self.side_vec * self.width
        right_rear = self.pos - self.forward_vec * self.length - self.side_vec * self.width

        pygame.draw.polygon(screen, self.color, [tip.pair(), left_rear.pair(), right_rear.pair() ])
        self.probe.move()
        self.probe.draw(screen)

    def attack(self):
        for target in self.state_ref.targets:
            if self.pos.dist_to(target.pos) < self.length:
                self.state_ref.targets.remove(target)
                self.state_ref.actors.remove(target)
