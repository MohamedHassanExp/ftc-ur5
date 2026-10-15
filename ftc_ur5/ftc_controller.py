import numpy as np
from .robot_dynamics import UR5Dynamics

class AdaptiveSMCFTC:

    def __init__(self):
        self.dynamics = UR5Dynamics()
        self.n = 6
        self.Kp_base = np.array([50.0] * self.n)
        self.Kd_base = np.array([10.0] * self.n)
        self.dKp = np.zeros(self.n)
        self.dKd = np.zeros(self.n)
        self.eta_p = np.array([800.0] * self.n)
        self.eta_d = np.array([200.0] * self.n)
        self.e_dead = 0.005
        self.de_dead = 0.010
        self.dKp_max = np.array([300.0] * self.n)
        self.dKd_max = np.array([100.0] * self.n)
        self.dt = 0.001
        self.K_history = []

    def compute(self, q, dq, q_d, dq_d, ddq_d):
        e = q_d - q
        de = dq_d - dq
        Kp_eff = self.Kp_base + self.dKp
        Kd_eff = self.Kd_base + self.dKd
        M = self.dynamics.inertia_matrix(q)
        C = self.dynamics.coriolis_matrix(q, dq)
        G = self.dynamics.gravity_vector(q)
        ddq_cmd = ddq_d + Kd_eff * de + Kp_eff * e
        tau = np.clip(M @ ddq_cmd + C @ dq + G, -150.0, 150.0)
        for i in range(self.n):
            if abs(e[i]) > self.e_dead:
                self.dKp[i] += self.eta_p[i] * abs(e[i]) * self.dt
            if abs(de[i]) > self.de_dead:
                self.dKd[i] += self.eta_d[i] * abs(de[i]) * self.dt
        self.dKp = np.clip(self.dKp, 0.0, self.dKp_max)
        self.dKd = np.clip(self.dKd, 0.0, self.dKd_max)
        self.K_history.append(self.dKp.copy())
        return tau

    def reset(self):
        self.dKp = np.zeros(self.n)
        self.dKd = np.zeros(self.n)
        self.K_history = []
