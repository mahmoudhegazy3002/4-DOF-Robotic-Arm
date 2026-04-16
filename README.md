# 4-DOF Robotic Arm: Kinematics, Trajectory Planning, and Multibody Simulation

## 📝 Overview
This repository contains the mathematical modeling, simulation, and trajectory planning for a 4 Degree-of-Freedom (4-DOF) robotic arm. The project integrates **MATLAB Simscape Multibody**, **Python**, and **Gazebo** to validate both the theoretical kinematics and the physical behavior of the manipulator.

The core mathematical foundation of this project includes the Denavit-Hartenberg (DH) conventions, forward/inverse position kinematics, velocity kinematics (Jacobian), and smooth trajectory generation.

## 👨‍💻 My Contribution: MATLAB Simscape Multibody
While this project was a collaborative mechatronics engineering effort utilizing Python and Gazebo for system integration, **my primary responsibility was the MATLAB Multibody development**. 

My work focused on bridging the mathematical models with physical simulations, specifically:
* **Simscape Multibody Modeling:** Translating the physical parameters of the 4-DOF arm into a robust MATLAB Multibody simulation environment.
* **Kinematic Validation:** Verifying the calculated DH parameters, position kinematics, and velocity kinematics against the physical simulation.
* **Trajectory Execution:** Simulating the planned trajectories within MATLAB to analyze motion profiles before hardware/Gazebo deployment.

## 🛠️ Tech Stack & Tools
* **MATLAB & Simulink (Simscape Multibody):** Physical modeling, kinematic verification, and dynamic analysis.
* **Python:** Core scripting for trajectory generation and kinematics solvers.
* **Gazebo:** 3D rigid-body simulation for the robotic arm.

## 🧮 Mathematical Modeling
The theoretical foundation of the robotic arm includes:
1. **DH Parameters:** Complete Denavit-Hartenberg table formulation to define the relationship between joint frames.
2. **Position Kinematics:** Forward kinematics to determine end-effector pose, and Inverse kinematics to calculate required joint angles.
3. **Velocity Kinematics:** Derivation of the Jacobian matrix to map joint velocities to end-effector velocities.
4. **Trajectory Planning:** Generation of smooth point-to-point and continuous path trajectories.

## 🚀 Installation & Usage

### Prerequisites
* MATLAB (with Simscape Multibody Add-on)
* ROS 2 / Gazebo
* Python 3.x 

### Running the MATLAB Simulation
1. Clone this repository:
   ```bash
   git clone [https://github.com/yourusername/your-repo-name.git](https://github.com/yourusername/your-repo-name.git)
