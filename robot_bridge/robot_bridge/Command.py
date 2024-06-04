#!/usr/bin/python3

import numpy as np

class Command():
    def __init__(self, r : float, b : float):
        self.Tt2w = np.array([1.0 / r, -0.5 * b / r,
                              1.0 / r, 0.5 * b / r]).reshape(2, 2)
        self.Tw2t = np.array([0.5 * r, 0.5 * r,
                              0.0, 0.0, 
                              -r / b, r / b]).reshape(3, 2)
        self.pos_g = np.zeros(3)
        self.vel_g = np.zeros(3)

    def get_wheelspeed(self, vel_r : list):
        return self.Tt2w @ np.array(vel_r)
    
    def get_pose(self, qd : np, dt : float):
        Rr2g = np.array([np.cos(self.pos_g[2]), -np.sin(self.pos_g[2]), 0.0,
                         np.sin(self.pos_g[2]), np.cos(self.pos_g[2]), 0.0,
                         0.0, 0.0, 1.0]).reshape(3, 3)
        self.pos_g += (Rr2g @ self.Tw2t @ qd) * dt
        return self.pos_g

    def get_twist(self, qd : np):
        return np.linalg.inv(self.Tt2w) @ qd
    
    def get_pose_trapezoidal(self, qd : np, dt : float):
        Rr2g = np.array([np.cos(self.pos_g[2]), -np.sin(self.pos_g[2]), 0.0,
                         np.sin(self.pos_g[2]), np.cos(self.pos_g[2]), 0.0,
                         0.0, 0.0, 1.0]).reshape(3, 3)
        vel_g = (Rr2g @ self.Tw2t @ qd)
        self.pos_g += 0.5 * (vel_g + self.vel_g) * dt
        self.vel_g = vel_g
        return self.pos_g