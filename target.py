import pygame
import math

from actor import Actor
from circle import Circle
from vector import Vector

from constants import BORDER_BUFFER

class Target(Actor):
    def __init__(self, state_ref):
        super().__init__(state_ref)
        self.max_speed = 3.5 
        self.length = 10
        self.probe_distance = 30
        self.probe = Circle(self, self.state_ref, self.probe_distance)
        self.theta_range = 0.1 
        self.theta = 0

    def move(self):
        forward_vec = self.forward_vec       
        self.theta += self._rng.uniform(-self.theta_range, self.theta_range)
        
        circle_center = self.pos + forward_vec * 50
        
        current_angle = math.atan2(forward_vec.y, forward_vec.x)
        target_angle = self.theta + current_angle
        
        displacement = Vector(
            math.cos(target_angle) * 50,    
            math.sin(target_angle) * 50
        )
        
        wander_target = circle_center + displacement
        
        desired_vel = (wander_target - self.pos).normalize() * self.max_speed
        steering_direction = desired_vel - self.vel
        
        self.steering_force = steering_direction.truncate(self.max_force)
        self.acc = self.steering_force / self.mass
        self.vel = (self.vel + self.acc).truncate(self.max_speed)
        self.pos += self.vel
        
        if self.encounter_border():
            print("Target encountered border, adjusting course.")
        self.reorient()

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), 10)
        self.probe.move()
        self.probe.draw(screen)

    def encounter_border(self):
        w, h = self.state_ref.width, self.state_ref.height
        if self.pos.x < 0:
            self.pos.x = 0
            self.vel.x *= -1
        elif self.pos.x > w:
            self.pos.x = w
            self.vel.x *= -1
        if self.pos.y < 0:
            self.pos.y = 0
            self.vel.y *= -1
        elif self.pos.y > h:
            self.pos.y = h
            self.vel.y *= -1