# Moveit_Dual_FR3

A ROS 2 package for controlling dual [Franka Emika FR3](https://franka.de/fr3) collaborative robot arms using [MoveIt2](https://moveit.ros.org/) and [MoveIt Task Constructor (MTC)](https://github.com/ros-planning/moveit_task_constructor) for task-based manipulation.

It is intented to run inside a docker container. Check docker image and container setup from following link.

https://hub.docker.com/repository/docker/ericmjk/moveit_dual_fr3/general

## Overview

This package implements a dual-arm pick-and-handover demo: the right arm picks an object, passes it to the left arm, and the left arm places it at a target location. It also provides Python scripting utilities for joint-space and Cartesian motion control.

```
Moveit_Dual_FR3/
├── dual_arm/           # Main ROS 2 package (C++ MTC demo + configuration)
│   ├── src/            # mtc_handover.cpp — main handover executable
│   ├── include/        # dual_arm_task.hpp — task class definition
│   ├── config/         # URDF/SRDF xacros, kinematics, controllers, OMPL config
│   ├── launch/         # Launch files and shared MoveIt config module
│   └── rviz/           # RViz visualization configs
└── scripts/            # Python scripting utilities package
    └── scripts/        # arm_mover.py, joint_goal.py, pose_goal.py
```

## Prerequisites

- **ROS 2 Humble** (or compatible)
- **MoveIt2**
- **MoveIt Task Constructor**
- **franka_ros2** (for real hardware)

## Building

```bash
cd ~/ros2_ws/src
git clone https://github.com/eric-mjk/Moveit_Dual_FR3.git
cd ~/ros2_ws
colcon build --packages-select dual_arm scripts
source install/setup.bash
```

## Robot Configuration

Two FR3 arms share a common `world` frame:

| Arm   | Planning Group   | Y Offset | End-Effector Link       |
|-------|------------------|----------|-------------------------|
| Left  | `left_fr3_arm`   | −0.5 m   | `left_fr3_hand_tcp`     |
| Right | `right_fr3_arm`  | +0.5 m   | `right_fr3_hand_tcp`    |
| Both  | `both_arms`      | —        | —                       |

**Kinematics:** LMA (Levenberg-Marquardt) IK solver, 0.005 rad search resolution.
**Motion Planning:** OMPL (`RRTConnectkConfigDefault` default, 25+ algorithms available).

## Usage

### 1. Launch the Full MoveIt Stack

**Simulation (fake hardware):**
```bash
ros2 launch dual_arm format_moveit.launch.py use_fake_hardware:=true
```

**Without grippers:**
```bash
ros2 launch dual_arm format_moveit.launch.py use_fake_hardware:=true load_gripper:=false
```

**Real hardware:**
```bash
ros2 launch dual_arm format_moveit.launch.py \
  left_robot_ip:=<LEFT_IP> \
  right_robot_ip:=<RIGHT_IP> \
  use_fake_hardware:=false
```

### 2. Run the MTC Handover Demo

With the full MoveIt stack already running:
```bash
ros2 launch dual_arm mtc_moveit.launch.py
```

The demo executes the following sequence:
1. Both arms move to `ready` position
2. **Right arm** approaches, grasps, and lifts the object
3. **Handover** — left arm approaches and takes the object from the right
4. **Left arm** carries the object to the place pose and releases it
5. Left arm returns home

### 3. Python Scripting Utilities

These scripts use MoveItPy and require the full MoveIt stack to be running.

**Joint-space motion:**
```bash
ros2 run scripts joint_goal \
  --group left_fr3_arm \
  --joints 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785 \
  --vel 0.5
```

**Named state (SRDF):**
```bash
ros2 run scripts joint_goal --group right_fr3_arm --named ready
```

**Both arms simultaneously (14 joint values: left then right):**
```bash
ros2 run scripts joint_goal --group both_arms --joints <j1..j7_left> <j1..j7_right>
```

**Cartesian (pose) goal — single arm only:**
```bash
ros2 run scripts pose_goal \
  --group left_fr3_arm \
  --pos 0.4 0.0 0.5 \
  --quat 0.0 0.0 0.0 1.0 \
  --vel 0.3
```

#### Script Arguments

| Argument  | Description                          | Default              |
|-----------|--------------------------------------|----------------------|
| `--group` | Planning group                       | `left_fr3_arm`       |
| `--ee`    | End-effector link                    | `left_fr3_hand_tcp`  |
| `--frame` | Reference frame                      | `world`              |
| `--vel`   | Velocity scaling (0.0–1.0)           | `0.5`                |
| `--time`  | Planning timeout (seconds)           | `5.0`                |
| `--named` | SRDF named state (`ready`, `extended`) | —                  |
| `--joints`| Joint angles in radians              | —                    |
| `--pos`   | XYZ position (pose goal only)        | `0.4 0.0 0.5`        |
| `--quat`  | Quaternion xyzw (pose goal only)     | `0.0 0.0 0.0 1.0`    |

> **Note:** `both_arms` group is not supported with `pose_goal`.

### 4. Script Launcher

Run scripts with full MoveIt configuration via launch file:
```bash
ros2 launch dual_arm run_script.launch.py \
  script:=joint_goal \
  group:=left_fr3_arm \
  named:=ready
```

## Controllers

| Mode        | Command Interface | Update Rate |
|-------------|-------------------|-------------|
| Real HW     | Effort            | 1000 Hz     |
| Fake/Sim    | Position          | 1000 Hz     |

Trajectory execution: 1.2× duration scaling, 0.5 s goal time margin, 0.05 s start tolerance.

## Package Structure Details

### `dual_arm` (C++ / CMake)

| File | Description |
|------|-------------|
| `src/mtc_handover.cpp` | Main MTC handover demo |
| `include/dual_arm/dual_arm_task.hpp` | `DualArmTask` and `RobotConfig` / `PickPlaceParams` definitions |
| `scripts/fake_gripper_action_server.py` | Simulated `GripperCommand` action server for testing |
| `config/dual_fr3.urdf.xacro` | Dual-arm URDF (left at Y=−0.5, right at Y=+0.5) |
| `config/dual_fr3.srdf.xacro` | Dual-arm SRDF with collision pairs |
| `config/dual_kinematics.yaml` | Dual-arm IK solver settings |
| `config/ompl_planning.yaml` | OMPL planner configurations |
| `config/joint_limits.yaml` | Per-joint position, velocity, effort limits |

### `scripts` (Python / setuptools)

| File | Description |
|------|-------------|
| `arm_mover.py` | `ArmMover` class wrapping MoveItPy for motion planning and execution |
| `joint_goal.py` | CLI for joint-space and named-state goals |
| `pose_goal.py` | CLI for Cartesian end-effector goals |

## License

Apache-2.0 — see [LICENSE](LICENSE).

## Maintainer

Eric Kim — eric.mjkim35@gmail.com
