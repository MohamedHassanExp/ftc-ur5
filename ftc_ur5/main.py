from .simulation import Simulation

def main():
    print("=" * 50)
    print("FTC vs PID — UR5 Fault Tolerance Simulation")
    print("ROS 2 Lyrical | Ubuntu 26.04 Resolute")
    print("=" * 50)
    sim = Simulation()
    results = sim.run_all_scenarios()
    print("\nSimulation complete. Figures saved to /tmp/")

if __name__ == "__main__":
    main()
