# UR5 Fault-Tolerant Control (FTC)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=flat-square)](https://opensource.org/licenses/MIT)
[![Python Version](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/)

A Python-based robotics package implementing fault-tolerant control (FTC) algorithms, dynamic modeling, and simulation for the Universal Robots UR5 manipulator.

##  Table of Contents
- [About the Project](#about-the-project)
- [Tech Stack](#tech-stack)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)

##  About the Project

This project provides a simulation environment and control architecture for testing fault-tolerant strategies on a UR5 robotic arm. It includes baseline PID control, custom fault injection models, and FTC logic to maintain system stability and trajectory tracking despite simulated actuator or sensor failures.

*   **Language:** Python
*   **Framework:** ROS 2 (Robot Operating System)
*   **Domain:** Robot Dynamics, Fault-Tolerant Control, Simulation

##  Getting Started

Follow these instructions to view or modify the hardware design files locally.

### Prerequisites

*   Python 3.8+
*   ROS 2 (Humble or newer recommended)
*   Colcon (for building the workspace)

### Installation

1. Clone the repository into your ROS 2 workspace `src` directory:
   ```bash
   git clone https://github.com/MohamedHassanExp/ftc-ur5.git
2. Navigate to your workspace root and build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ftc_ur5
3. Source the setup file:
   ```bash
   source install/setup.bash

## Repository Structure

The core logic is organized within the ftc_ur5 package directory:

*    robot_dynamics.py: Mathematical models representing the kinematics and dynamics of the UR5 manipulator.
*    pid_controller.py: Standard Proportional-Integral-Derivative baseline controller.
*    ftc_controller.py: Advanced control logic designed to compensate for detected hardware faults.
*    fault_model.py: Framework for injecting simulated hardware failures into the system during testing.
*    simulation.py & main.py: Entry points for running the control loops and dynamic simulations.
*    package.xml, setup.py, setup.cfg: Standard ROS 2 and Python package configuration and dependency management files.
