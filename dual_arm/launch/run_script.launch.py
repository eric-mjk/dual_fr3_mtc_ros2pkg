"""
run_script.launch.py
--------------------
Runs joint_goal or pose_goal from the scripts package with the full
dual_arm MoveIt configuration loaded onto the node's parameter server.

MoveItPy requires robot_description, robot_description_semantic,
kinematics, and planning pipeline parameters on its own node —
they are NOT inherited from the running move_group node.

All MoveIt config is defined in moveit_config.py (single source of truth).
When you change the robot config, only edit moveit_config.py.

Prerequisites:
  ros2 launch dual_arm refactor_moveit.launch.py use_fake_hardware:=true

Usage:
  # Named state
  ros2 launch dual_arm run_script.launch.py script:=joint_goal named:=ready

  # Explicit joint angles (7 values, radians)
  ros2 launch dual_arm run_script.launch.py script:=joint_goal \
      joints:="0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"

  # Both arms (14 values, left j1-j7 first then right j1-j7)
  ros2 launch dual_arm run_script.launch.py script:=joint_goal \
      group:=both_arms \
      joints:="0.0 -0.785 0.0 -2.356 0.0 1.571 0.785 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"

  # Pose goal
  ros2 launch dual_arm run_script.launch.py script:=pose_goal \
      group:=right_fr3_arm ee:=right_fr3_hand_tcp \
      pos:="0.4 0.3 0.5" quat:="0.0 0.0 0.0 1.0"
"""

import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_config import declare_launch_args, get_moveit_params


def generate_launch_description():
    # ------------------------------------------------------------------
    # Script-specific launch args
    # ------------------------------------------------------------------
    script_args = [
        DeclareLaunchArgument("script",  default_value="joint_goal",
                              description="Script to run: joint_goal or pose_goal"),
        DeclareLaunchArgument("group",   default_value="left_fr3_arm",
                              description="Planning group: left_fr3_arm | right_fr3_arm | both_arms"),
        DeclareLaunchArgument("ee",      default_value="left_fr3_hand_tcp",
                              description="End-effector link (ignored for both_arms)"),
        DeclareLaunchArgument("frame",   default_value="world",
                              description="Reference frame for pose goals"),
        DeclareLaunchArgument("vel",     default_value="0.3",
                              description="Velocity scale 0-1"),
        DeclareLaunchArgument("time",    default_value="5.0",
                              description="Max planning time (seconds)"),
        # joint_goal
        DeclareLaunchArgument("named",   default_value="",
                              description="Named SRDF state: ready | extended"),
        DeclareLaunchArgument("joints",  default_value="",
                              description="Space-separated joint angles in radians"),
        # pose_goal
        DeclareLaunchArgument("pos",     default_value="0.4 0.0 0.5",
                              description="Target position 'X Y Z'"),
        DeclareLaunchArgument("quat",    default_value="0.0 0.0 0.0 1.0",
                              description="Target orientation 'QX QY QZ QW'"),
    ]

    # ------------------------------------------------------------------
    # MoveIt config (single source of truth: moveit_config.py)
    # ------------------------------------------------------------------
    hw_args, lc = declare_launch_args()
    moveit_params = get_moveit_params(lc)

    # ------------------------------------------------------------------
    # Script node
    # ------------------------------------------------------------------
    script_node = Node(
        package="scripts",
        executable=LaunchConfiguration("script"),
        name="arm_script_node",
        output="screen",
        parameters=moveit_params,
        arguments=[
            "--group",  LaunchConfiguration("group"),
            "--ee",     LaunchConfiguration("ee"),
            "--frame",  LaunchConfiguration("frame"),
            "--vel",    LaunchConfiguration("vel"),
            "--time",   LaunchConfiguration("time"),
            "--named",  LaunchConfiguration("named"),
            "--pos",    LaunchConfiguration("pos"),
            "--quat",   LaunchConfiguration("quat"),
            "--joints", LaunchConfiguration("joints"),
        ],
    )

    return LaunchDescription(script_args + hw_args + [script_node])
