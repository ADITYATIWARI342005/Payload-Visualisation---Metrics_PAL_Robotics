# ROS 2 Pinocchio Node
This repository contains a ROS 2 node that demonstrates using the Pinocchio Rigid Body Dynamics Library with a robot description. The node subscribes to a robot description (URDF) and joint states, then performs various operations using Pinocchio.

## Features

This node demonstrates several key capabilities of the Pinocchio library:

1. **Creating a Pinocchio Model**: Parses a URDF description into a Pinocchio model
2. **Forward Kinematics**: Computes transforms for all frames in the robot
3. **Jacobian Computation**: Calculates the Jacobian matrix at a specific frame (end effector)
4. **Collision Detection**: Performs basic collision checking between robot links

## My Setup Specifications
- ROS 2 Humble
- Pinocchio library
- C++17 compatible compiler

## Installation

### Install ROS 2
Follow the [official ROS 2 installation instructions](https://docs.ros.org/en/humble/Installation.html).

### Install Pinocchio
```bash
sudo apt install ros-$ROS2_Humble-pinocchio
```

### Building from Source
```bash
# Create a workspace
mkdir -p ~/ros2_pinocchio_ws/src
cd ~/ros2_pinocchio_ws/src

# Clone this repository
git clone https://github.com/username/ros2_pinocchio_node.git

# Build the workspace
cd ~/ros2_pinocchio_ws
colcon build
```

## Usage

### Launching the Node

```bash
# Source the workspace
source ~/ros2_pinocchio_ws/install/setup.bash

# Launch the node
ros2 launch ros2_pinocchio_node robot_model.launch.py
```

### Publishing Test Joint States

The repository includes a test script to publish joint states for testing.

```bash
# In a separate terminal
source ~/ros2_pinocchio_ws/install/setup.bash
ros2 run ros2_pinocchio_node test_joint_states.py
```

## Node Parameters

- `urdf_param_name`: Parameter name for the robot description (default: "robot_description")
- `base_link`: Name of the robot's base link (default: "base_link")
- `end_effector_link`: Name of the end effector link (default: "tool0")
- `collision_link1`: First link for collision checking (default: "link1")
- `collision_link2`: Second link for collision checking (default: "link6")

## Expected Output

When running the node, you should see output similar to:

```
[INFO] [robot_model_node]: Robot Model Node initialized
[INFO] [robot_model_node]: Received robot description, creating Pinocchio model
[INFO] [robot_model_node]: Successfully created Pinocchio model with 6 DoF
[INFO] [robot_model_node]: Found end effector frame: tool0 (ID: 14)
[INFO] [robot_model_node]: Setting up collision model
[INFO] [robot_model_node]: Collision model created successfully
[INFO] [robot_model_node]: Processing joint state
[INFO] [robot_model_node]: Test 1: Performing forward kinematics
[INFO] [robot_model_node]: End effector position: [0.352415, 0.123456, 0.871012]
[INFO] [robot_model_node]: Test 2: Computing Jacobian at end effector
[INFO] [robot_model_node]: Jacobian (first 3x3 block):
[INFO] [robot_model_node]: 0 0.1 0.2 
[INFO] [robot_model_node]: 0.3 0.4 0.5 
[INFO] [robot_model_node]: 0.6 0.7 0.8 
[INFO] [robot_model_node]: Test 3: Checking collisions between links
[INFO] [robot_model_node]: Distance between 'link1' and 'link6' is 0.823456 meters
[INFO] [robot_model_node]: No collision detected between links
```

The exact values will depend on your robot model and joint positions.

## Project Structure

```
ros2_pinocchio_node/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── robot_model_node.cpp
├── launch/
│   └── robot_model.launch.py
├── urdf/
│   └── robot.urdf
└── test/
    └── test_joint_states.py
```

## Implementation Details

### Main Components

1. **RobotModelNode Class**: The primary C++ class that handles all functionality
2. **URDF Subscription**: Subscribes to the robot description and builds a Pinocchio model
3. **Joint State Subscription**: Processes joint states to update the robot state
4. **Forward Kinematics**: Computes transforms for all links
5. **Jacobian Computation**: Calculates the Jacobian at the end effector
6. **Collision Detection**: Simple collision checking between links

### Key Functions

- `urdf_callback`: Processes the URDF string and builds the Pinocchio model
- `joint_state_callback`: Handles incoming joint states
- `performForwardKinematics`: Runs the forward kinematics algorithm
- `computeJacobian`: Calculates the Jacobian matrix
- `checkCollisions`: Performs collision detection
