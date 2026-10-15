import numpy as np

class UR5Dynamics:

    def __init__(self):
        self.m = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879]
        self.L = [0.089, 0.425, 0.392, 0.109, 0.095, 0.082]
        self.g = 9.81
        self.n = 6

    def inertia_matrix(self, q):
        M = np.zeros((self.n, self.n))
        for i in range(self.n):
            M[i, i] = sum(self.m[j] * self.L[j]**2
                         for j in range(i, self.n)) + 0.1
        return M

    def coriolis_matrix(self, q, dq):
        C = np.zeros((self.n, self.n))
        for i in range(self.n):
            for j in range(self.n):
                C[i, j] = 0.01 * dq[j]
        return C

    def gravity_vector(self, q):
        G = np.zeros(self.n)
        for i in range(self.n):
            G[i] = sum(self.m[j] * self.g * self.L[j] * np.cos(q[i])
                      for j in range(i, self.n)) * 0.1
        return G

    def forward_dynamics(self, q, dq, tau):
        M = self.inertia_matrix(q)
        C = self.coriolis_matrix(q, dq)
        G = self.gravity_vector(q)
        ddq = np.linalg.solve(M, tau - C @ dq - G)
        return ddq

    def inverse_dynamics(self, q, dq, ddq):
        M = self.inertia_matrix(q)
        C = self.coriolis_matrix(q, dq)
        G = self.gravity_vector(q)
        return M @ ddq + C @ dq + G
