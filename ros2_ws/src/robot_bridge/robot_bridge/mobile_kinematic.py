#!/usr/bin/python3

import numpy as np
from ackermann_interfaces.msg import AckermannFeedback

class Ackerman_Kinematic():
    def __init__(self, r : float, b : float, a : float):

        self.Tt2w = np.array([1.0 / r, -0.5 * b / r,
                              1.0 / r, 0.5 * b / r]).reshape(2, 2)
        self.Tw2t = np.array([0.5 * r, 0.5 * r,
                              0.0, 0.0, 
                              -r / b, r / b]).reshape(3, 2)
        self.pos_g = np.zeros(3)
        self.vel_g = np.zeros(3)

        self.axle_length = a
        self.wheelbase_length = b
        self.wheel_radius = r
        self.center_of_mass_offset = c
        self.damping_factor = d

    def get_wheelspeed(self, linear_speed, angular_speed):
        
        return self.Tt2w @ np.array(vel_r)
    
    def get_pose(self, qd : np, dt : float):
        Rr2g = np.array([np.cos(self.pos_g[2]), -np.sin(self.pos_g[2]), 0.0,
                         np.sin(self.pos_g[2]), np.cos(self.pos_g[2]), 0.0,
                         0.0, 0.0, 1.0]).reshape(3, 3)
        self.pos_g += (Rr2g @ self.Tw2t @ qd) * dt
        return self.pos_g

    def linear_velocity(self, orientation, speed):
        return orientation.apply([speed, 0, 0])
    
    def turn_radius(self, steering_angle):
        if steering_angle == 0:
            return np.inf
        else:
            radius = np.sqrt((self.center_of_mass_offset**2 + self.wheelbase_length**2) * 1 / np.tan(steering_angle)**2)
            return np.copysign(radius, steering_angle)

    def get_twist(self, wheel_speed, steering_angle, orientation):
        linear_speed = self.wheel_radius * wheel_speed
        linear_velocity = self.linear_velocity(orientation, linear_speed)
        angular_speed = linear_speed / self.turn_radius(steering_angle)

        return [linear_velocity, angular_speed]
    
    def get_pose_trapezoidal(self, qd : np, dt : float):
        Rr2g = np.array([np.cos(self.pos_g[2]), -np.sin(self.pos_g[2]), 0.0,
                         np.sin(self.pos_g[2]), np.cos(self.pos_g[2]), 0.0,
                         0.0, 0.0, 1.0]).reshape(3, 3)
        vel_g = (Rr2g @ self.Tw2t @ qd)
        self.pos_g += 0.5 * (vel_g + self.vel_g) * dt
        self.vel_g = vel_g
        return self.pos_g