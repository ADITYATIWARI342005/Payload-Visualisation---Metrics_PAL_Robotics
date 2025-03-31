# ROS 2 Setup Guide
## project structure
1_Describe the step you need to have ROS 2 in your PCs/
│-- steps.txt          # Text file with installation steps
│-- Screenshot1.png    # Screenshot reference
│-- Screenshot2.png    # Additional visual reference

## Overview
This directory provides step-by-step instructions for setting up **ROS 2 Humble** on Ubuntu 22.04. It includes text-based installation instructions and screenshots for reference.

---

## Installation Steps
To install ROS 2 Humble, follow the steps provided in the `steps.txt` file. Below is a summarized installation process:

### 1. Update and Upgrade System
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Add ROS 2 Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### 3. Install ROS 2 Humble
```bash
sudo apt install ros-humble-desktop
```
For a minimal installation, use:
```bash
sudo apt install ros-humble-ros-base
```

### 4. Setup Environment
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Install Dependencies (Optional)
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```
Initialize `rosdep`:
```bash
sudo rosdep init
rosdep update
```

---

## Screenshots
The `Screenshot` files in this directory provide visual references for installation steps. If you encounter any issues, refer to them for guidance.

---

## Verification
To verify your installation, run:
```bash
ros2 run demo_nodes_cpp talker
```
If the installation is successful, you should see messages being published.

---

## Troubleshooting
- Ensure that your system is fully updated before installation.
- If `rosdep update` fails, check your internet connection and retry.
- For any missing dependencies, use:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

---

## Additional Resources
- Official ROS 2 Installation Guide: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

---



