import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from .robot_dynamics import UR5Dynamics
from .fault_model import ActuatorFaultModel
from .pid_controller import PIDController
from .ftc_controller import AdaptiveSMCFTC


class Simulation:

    def __init__(self):
        self.dt = 0.001
        self.T = 15.0
        self.t_fault = 5.0
        self.n = 6

    def desired_trajectory(self, t):
        freq = 0.2
        amp = np.array([0.5, 0.4, 0.3, 0.4, 0.3, 0.2])
        q_d = amp * np.sin(2 * np.pi * freq * t)
        dq_d = amp * 2 * np.pi * freq * np.cos(2 * np.pi * freq * t)
        ddq_d = -amp * (2 * np.pi * freq)**2 * np.sin(2 * np.pi * freq * t)
        return q_d, dq_d, ddq_d

    def rk4_step(self, dynamics, q, dq, tau, dt):
        def f(q, dq):
            return dynamics.forward_dynamics(q, dq, tau)
        k1 = f(q, dq)
        k2 = f(q + 0.5*dt*dq, dq + 0.5*dt*k1)
        k3 = f(q + 0.5*dt*dq, dq + 0.5*dt*k2)
        k4 = f(q + dt*dq,     dq + dt*k3)
        ddq = (k1 + 2*k2 + 2*k3 + k4) / 6.0
        dq_new = dq + ddq * dt
        q_new = q  + dq  * dt
        return q_new, dq_new

    def run(self, controller, fault_model, label=""):
        dynamics = UR5Dynamics()
        steps = int(self.T / self.dt)
        t_arr = np.linspace(0, self.T, steps)
        q = np.zeros(self.n)
        dq = np.zeros(self.n)
        log_every = 10
        log_steps = steps // log_every
        q_log = np.zeros((log_steps, self.n))
        qd_log = np.zeros((log_steps, self.n))
        e_log = np.zeros((log_steps, self.n))
        log_t = np.zeros(log_steps)
        log_i = 0
        fault_injected = False

        for i, t in enumerate(t_arr):
            if t >= self.t_fault and not fault_injected:
                fault_injected = True
                print(f"[{label}] Fault injected at t={t:.3f}s")
                for s in fault_model.get_status():
                    print(f"  {s}")
            q_d, dq_d, ddq_d = self.desired_trajectory(t)
            tau_cmd = controller.compute(q, dq, q_d, dq_d, ddq_d)
            tau_actual = fault_model.apply(tau_cmd) if fault_injected else tau_cmd
            tau_actual = np.clip(tau_actual, -150.0, 150.0)
            q, dq = self.rk4_step(dynamics, q, dq, tau_actual, self.dt)
            if i % log_every == 0 and log_i < log_steps:
                q_log[log_i] = q
                qd_log[log_i] = q_d
                e_log[log_i] = q_d - q
                log_t[log_i] = t
                log_i += 1

        t_log = log_t[:log_i]
        q_log = q_log[:log_i]
        qd_log = qd_log[:log_i]
        e_log = e_log[:log_i]
        rmse = np.sqrt(np.mean(e_log**2, axis=0))
        print(f"[{label}] RMSE: {np.round(rmse, 5)}")
        return t_log, q_log, qd_log, e_log, rmse

    def run_all_scenarios(self):
        results = {}

        print("\n=== Scenario S1: Fault-Free ===")
        f0a = ActuatorFaultModel()
        f0b = ActuatorFaultModel()
        ftc1 = AdaptiveSMCFTC()
        t, q_pid, qd, e_pid, r_pid = self.run(PIDController(), f0a, "PID fault-free")
        _, q_ftc, _,  e_ftc, r_ftc = self.run(ftc1, f0b, "FTC fault-free")
        results["s1"] = (t, q_pid, q_ftc, qd, e_pid, e_ftc, r_pid, r_ftc)

        print("\n=== Scenario S2: Single Joint Fault (Joint 2, LOE=0.5) ===")
        f1a = ActuatorFaultModel(); f1a.set_fault(1, rho=0.5)
        f1b = ActuatorFaultModel(); f1b.set_fault(1, rho=0.5)
        ftc2 = AdaptiveSMCFTC()
        t2, q_pid2, qd2, e_pid2, r_pid2 = self.run(PIDController(), f1a, "PID single")
        _,  q_ftc2, _,   e_ftc2, r_ftc2 = self.run(ftc2, f1b, "FTC single")
        results["s2"] = (t2, q_pid2, q_ftc2, qd2, e_pid2, e_ftc2, r_pid2, r_ftc2)

        print("\n=== Scenario S3: Simultaneous Multi-Joint Fault ===")
        f2a = ActuatorFaultModel()
        f2b = ActuatorFaultModel()
        f2a.set_multi_fault({0:(0.5,0.0), 1:(0.6,0.5), 2:(0.7,0.0)})
        f2b.set_multi_fault({0:(0.5,0.0), 1:(0.6,0.5), 2:(0.7,0.0)})
        ftc3 = AdaptiveSMCFTC()
        t3, q_pid3, qd3, e_pid3, r_pid3 = self.run(PIDController(), f2a, "PID multi")
        _,  q_ftc3, _,   e_ftc3, r_ftc3 = self.run(ftc3, f2b, "FTC multi")
        results["s3"] = (t3, q_pid3, q_ftc3, qd3, e_pid3, e_ftc3, r_pid3, r_ftc3, ftc3)

        self.plot_results(results)
        return results

    def plot_results(self, results):
        jl = [f"Joint {i+1}" for i in range(6)]
        t,  q_pid,  q_ftc,  qd,  e_pid,  e_ftc,  r_pid,  r_ftc              = results["s1"]
        t2, q_pid2, q_ftc2, qd2, e_pid2, e_ftc2, r_pid2, r_ftc2             = results["s2"]
        t3, q_pid3, q_ftc3, qd3, e_pid3, e_ftc3, r_pid3, r_ftc3, ftc3       = results["s3"]

        # Figure 1: Fault-free
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle("Figure 1: Fault-Free Trajectory Tracking", fontsize=14, fontweight="bold")
        for i, ax in enumerate(axes.flat):
            ax.plot(t, qd[:,i],    "k--", label="Desired", lw=1.5)
            ax.plot(t, q_pid[:,i], "b-",  label="PID",     lw=1.2)
            ax.plot(t, q_ftc[:,i], "g-",  label="FTC",     lw=1.2, alpha=0.8)
            ax.axvline(5.0, color="r", ls=":", alpha=0.4)
            ax.set_title(jl[i]); ax.set_xlabel("Time (s)"); ax.set_ylabel("Position (rad)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig("/tmp/fig1_fault_free.png", dpi=150, bbox_inches="tight")
        print("Saved fig1_fault_free.png")

        # Figure 2: Single fault
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle("Figure 2: Single Joint Fault — Joint 2 (LOE=0.5)", fontsize=14, fontweight="bold")
        axes[0].plot(t2, qd2[:,1],    "k--", label="Desired",       lw=1.5)
        axes[0].plot(t2, q_pid2[:,1], "r-",  label="PID",           lw=1.2)
        axes[0].plot(t2, q_ftc2[:,1], "b-",  label="FTC (Proposed)",lw=1.2)
        axes[0].axvline(5.0, color="gray", ls=":", label="Fault injected")
        axes[0].set_ylabel("Joint 2 Position (rad)"); axes[0].legend(); axes[0].grid(True, alpha=0.3)
        axes[1].plot(t2, np.abs(e_pid2[:,1]), "r-", label="PID error", lw=1.2)
        axes[1].plot(t2, np.abs(e_ftc2[:,1]), "b-", label="FTC error", lw=1.2)
        axes[1].axvline(5.0, color="gray", ls=":")
        axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Tracking Error (rad)")
        axes[1].legend(); axes[1].grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig("/tmp/fig2_single_fault.png", dpi=150, bbox_inches="tight")
        print("Saved fig2_single_fault.png")

        # Figure 3: Multi-fault
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle("Figure 3: Simultaneous Multi-Joint Fault (Joints 1,2,3)", fontsize=14, fontweight="bold")
        for i, ax in enumerate(axes.flat):
            ax.plot(t3, qd3[:,i],    "k--", label="Desired", lw=1.5)
            ax.plot(t3, q_pid3[:,i], "r-",  label="PID",     lw=1.2)
            ax.plot(t3, q_ftc3[:,i], "b-",  label="FTC",     lw=1.2)
            ax.axvline(5.0, color="gray", ls=":", alpha=0.7)
            ax.set_title(jl[i]); ax.set_xlabel("Time (s)"); ax.set_ylabel("Position (rad)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig("/tmp/fig3_multi_fault.png", dpi=150, bbox_inches="tight")
        print("Saved fig3_multi_fault.png")

        # Figure 4: Adaptive gain evolution
        if ftc3.K_history:
            K_arr = np.array(ftc3.K_history)
            t_k = np.linspace(0, self.T, len(K_arr))
            fig, ax = plt.subplots(figsize=(12, 5))
            fig.suptitle("Figure 4: Adaptive Gain ΔKp Evolution", fontsize=14, fontweight="bold")
            for i in range(6):
                ax.plot(t_k, K_arr[:,i], label=jl[i], lw=1.2)
            ax.axvline(5.0, color="gray", ls=":", label="Fault injected")
            ax.set_xlabel("Time (s)"); ax.set_ylabel("Adaptive Gain ΔKp")
            ax.legend(); ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig("/tmp/fig4_adaptive_gain.png", dpi=150, bbox_inches="tight")
            print("Saved fig4_adaptive_gain.png")

        print("\nAll figures saved to /tmp/")
