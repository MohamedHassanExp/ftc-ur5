import numpy as np

class ActuatorFaultModel:
s
    def __init__(self, n_joints=6):
        self.n = n_joints
        self.rho = np.ones(n_joints)
        self.bias = np.zeros(n_joints)
        self.fault_active = np.zeros(n_joints, dtype=bool)

    def set_fault(self, joint_index, rho=1.0, bias=0.0):
        self.rho[joint_index] = rho
        self.bias[joint_index] = bias
        self.fault_active[joint_index] = True
        print(f"[FAULT] Joint {joint_index+1}: LOE={rho:.2f}, Bias={bias:.2f} Nm")

    def set_multi_fault(self, fault_dict):
        for joint_index, (rho, bias) in fault_dict.items():
            self.set_fault(joint_index, rho, bias)

    def clear_all(self):
        self.rho = np.ones(self.n)
        self.bias = np.zeros(self.n)
        self.fault_active = np.zeros(self.n, dtype=bool)

    def apply(self, tau_commanded):
        return self.rho * tau_commanded + self.bias

    def get_status(self):
        status = []
        for i in range(self.n):
            if self.fault_active[i]:
                status.append(f"Joint {i+1}: LOE={self.rho[i]:.2f}, Bias={self.bias[i]:.2f}Nm")
        return status if status else ["No active faults"]
