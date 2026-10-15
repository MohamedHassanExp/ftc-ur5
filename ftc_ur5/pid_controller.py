import numpy as np
from .robot_dynamics import UR5Dynamics

class PIDController:

    def __init__(self):
        self.dynamics = UR5Dynamics()
        self.n = 6
        self.Kp = np.array([50.0] * self.n)
        self.Kd = np.array([10.0] * self.n)
        self.dt = 0.01

    def compute(self, q, dq, q_d, dq_d, ddq_d):
        e = q_d - q
        de = dq_d - dq
        M = self.dynamics.inertia_matrix(q)
        C = self.dynamics.coriolis_matrix(q, dq)
        G = self.dynamics.gravity_vector(q)
        ddq_cmd = ddq_d + self.Kd * de + self.Kp * e
        tau = M @ ddq_cmd + C @ dq + G
        return np.clip(tau, -150.0, 150.0)

    def reset(self):
        pass
