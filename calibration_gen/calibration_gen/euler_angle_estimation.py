#!/usr/bin/python3

import numpy as np

class EulerAngle_Estimation():
    def __init__(self):
        self.euler_angle = np.zeros(3)

    def compute_angle(self, gx : float, gy : float, gz : float, dt : float):
        gyro = np.array([gx, gy, gz])
        c0 = 1.0
        c1 = np.sin(self.euler_angle[0]) * np.tan(self.euler_angle[1])
        c2 = np.cos(self.euler_angle[0]) * np.tan(self.euler_angle[1])
        c3 = 0.0
        c4 = np.cos(self.euler_angle[0])
        c5 = -np.sin(self.euler_angle[0])
        c6 = 0.0
        c7 = np.sin(self.euler_angle[0]) / np.cos(self.euler_angle[1])
        c8 = np.cos(self.euler_angle[0]) / np.cos(self.euler_angle[1])
        T = np.array([c0, c1, c2,
                      c3, c4, c5,
                      c6, c7, c8]).reshape(3, 3)
        self.euler_angle += dt * (T @ gyro)
        return self.euler_angle