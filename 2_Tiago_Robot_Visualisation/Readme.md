# TIAGo Robot Visualization

This repository provides tools to visualize the TIAGo robot from PAL Robotics in RViz. It uses the [tiago_robot](https://github.com/pal-robotics/tiago_robot) repository to load the robot description and display it in RViz.

## Project Structure

2_Tiago_Robot_Visualisation/
│-- Readme.md         # Instructions for setting up and running the visualization
│-- video_link        # A file containing a link to a demo video




## Dependencies

- ROS2 humble
- pal-robotics repositories:
  - tiago_robot

# TIAGo Robot Setup Guide for ROS 2 Humble

This guide explains how to set up and visualize the TIAGo robot using ROS 2 Humble.

## Installation

### Step 1: Create a New ROS 2 Workspace
First, create the workspace directory and initialize it:

```bash
mkdir -p ~/tiago_ws/src
cd ~/tiago_ws
colcon build
source install/setup.bash
```

To automatically source this workspace every time you open a terminal:

```bash
echo "source ~/tiago_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Clone the TIAGo Robot Repository
Navigate to the `src` folder and clone the repository:

```bash
cd ~/tiago_ws/src
git clone https://github.com/ADITYATIWARI342005/tiago_robot.git
```

### Step 3: Install Dependencies
Install the required dependencies using `rosdep`:

```bash
cd ~/tiago_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build the Workspace
Run the following command to build the workspace:

```bash
colcon build
```

### Step 5: Source the Workspace
Once the build is complete, source the workspace:

```bash
source install/setup.bash
```

### Step 6: Verify the `tiago_description` Package
Check that the package is detected:

```bash
ros2 pkg list | grep tiago_description
```

### Step 7: Install Additional Dependencies
If you encounter errors related to `launch_pal` or missing packages, install them:

```bash
sudo apt install ros-humble-launch-pal
```

### Step 8: Launch the TIAGo Robot in RViz
Run the launch command to visualize the robot in RViz:

```bash
ros2 launch tiago_description show.launch.py
```




## RViz Configuration

In RViz:
1. Set the Fixed Frame to "base_footprint"
2. Add a "RobotModel" display
3. Set the "Robot Description" parameter to "robot_description"
4. Save the configuration for future use

## Demo Video

For a video demonstration of how to use this package, please visit [TIAGo Robot Visualization Demo](https://youtu.be/OevigVy6m1Q).

## Customization

You can customize the TIAGo robot configuration by changing the parameters in the launch file:

- `robot`: Choose between "iron", "steel", or "titanium" models
- `arm`: Automatically set based on the robot model
- `end_effector`: Choose the end effector type
- `ft_sensor`: Enable/disable force-torque sensor
- `laser_model`: Choose the laser scanner model
- `camera_model`: Choose the camera model

