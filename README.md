# Payload Visualisation & Metrics - PAL Robotics

## Overview
This repository provides resources for setting up and visualizing the **Tiago robot** using **ROS 2 Humble**. It also includes steps to integrate the **Pinocchio** library for robot kinematics and dynamics. The repository is structured into three main sections:

1. **ROS 2 Setup:** Instructions to install and configure ROS 2.
2. **Tiago Robot Visualization:** Setup and launch files for visualizing the Tiago robot in RViz.
3. **Pinocchio Integration:** A ROS 2 package for integrating Pinocchio with Tiago.

---

## 1. ROS 2 Setup
### Installation Steps
The `1_Describe the step you need to have ROS 2 in your PCs` directory contains:
- **steps.txt** – A text file describing the installation process.
- **Screenshots** – Visual references for installation steps.

Follow the steps in `steps.txt` to set up ROS 2 Humble on Ubuntu 22.04.

---

## 2. Tiago Robot Visualization
This section provides files for visualizing the **Tiago** robot in **RViz**.

### Getting Started
1. Navigate to the `2_Tiago_Robot_Visualisation` directory.
2. Refer to the `Readme.md` file inside for specific instructions.
3. The `video_link` file contains a demo video link for guidance.

### Launching Tiago in RViz
```bash
ros2 launch tiago_description show.launch.py
```
Ensure that the `tiago_description` package is properly sourced before running the command.

---

## 3. Pinocchio Integration
This section contains a full ROS 2 package for **Pinocchio** integration.

### Package Contents
- **CMakeLists.txt** & **package.xml** – Define the ROS 2 package.
- **launch/** – ROS 2 launch files.
- **config/** – Configuration files.
- **src/** – Source code for integration.
- **urdf/** – Unified Robot Description Format (URDF) files.

### Installation & Build
1. Navigate to the `3_Pinnochio` directory.
2. Build the package using colcon:
   ```bash
   colcon build --packages-select pinocchio_robot
   ```
3. Source the setup file:
   ```bash
   source install/setup.bash
   ```
4. Run the launch file:
   ```bash
   ros2 launch pinocchio_robot some_launch_file.launch.py
   ```
   *(Replace `some_launch_file.launch.py` with the actual launch file name.)*

---

## Additional Notes
- Ensure all dependencies are installed before running the launch files.
- If any issues arise, refer to the README files inside individual directories.

---



