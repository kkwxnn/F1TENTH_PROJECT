#!/usr/bin/python3

import numpy as np

class Diff_Drive_Cov_Estimator():
    def __init__(self, kr : float, kl : float, r : float, b : float):
        self.k = np.array([kr, 0.0,
                           0.0, kl]).reshape(2, 2)
        self.p_cov = np.eye(3) * 1.0e-6
        self.r = r
        self.b = b
        
    def update_cov(self, theta : float, dqr : float, dql : float):
        dsr = dqr * self.r
        dsl = dql * self.r
        ds = (dsr + dsl) / 2.0
        dtheta = (dsr - dsl) / self.b
        s_cov = self.k.copy()
        s_cov[0][0] = s_cov[0][0] * np.abs(dsr)
        s_cov[1][1] = s_cov[1][1] *  np.abs(dsl)

        a0 = np.cos(theta + (dtheta / 2.0))
        a1 = np.sin(theta + (dtheta / 2.0))

        grad_pf = np.eye(3) * 1.0
        grad_pf[0][2] = -ds * a1
        grad_pf[1][2] = ds * a0

        grad_drlf = np.array([(0.5 * a0) - (0.5 * ds * a1 / self.b), (0.5 * a0) + (0.5 * ds * a1 / self.b),
                              (0.5 * a1) + (0.5 * ds * a0 / self.b), (0.5 * a1) - (0.5 * ds * a0 / self.b),
                              1.0 / self.b, -1.0 / self.b]).reshape(3, 2)
        
        self.p_cov = (grad_pf @ (self.p_cov @ grad_pf.T)) + (grad_drlf @ (s_cov @ grad_drlf.T))
        return self.p_cov
