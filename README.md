# README for FRCobot ROS 2 Package

## Overview

This repository contains ROS 2 packages designed for controlling the **Fairino3_v6** robot. The key components include robot description, MoveIt configuration, hardware interfaces, and control nodes. These packages provide an end-to-end solution for robot modeling, planning, and execution.

---

## Directory Structure

- **frcobot_ros2**: Contains the robot-related packages:
  - **fairino3_v6_moveit2_config**: MoveIt configuration for planning and control.
  - **fairino_description**: URDF and mesh files describing the robot's structure.
  - **fairino_hardware**: Interfaces and hardware-specific configuration for the robot.
  - **fairino_msgs**: Custom messages and services for the robot.
- **move_robot**: A node for moving the robot using MoveIt.

---

## Dependencies

- ROS2 **Humble**
- MoveIt 2
- Rviz 2
- ros2_controllers

---

## Building the Packages

1. Clone this repository into your ROS 2 workspace, typically ~/ros2_ws/src:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/jayprajapati009/frcobot_ros2_setup.git
    ```

2. Build the workspace:

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

---

## Launching and Testing the Packages

### Step 1: Launch the Goal Setter

```sh
ros2 launch fairino3_v6_moveit2_config goal_setter.launch.pys
```

This will:

- Load the robot's URDF and SRDF files.
- Set up the MoveIt planning interface.

### Step 2: Run the move_robot Node

```sh
ros2 run move_robot move_robot
```

This node:

- Sends the robot to a predefined target pose.
- Plans a trajectory using MoveIt and executes it.

---

## Refrences

[1] <https://github.com/FAIR-INNOVATION/frcobot_ros2>
[2] <https://fair-documentation.readthedocs.io/en/latest/index.html>
