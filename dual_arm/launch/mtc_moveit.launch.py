"""
mtc_moveit.launch.py
--------------------
Launches only the MTC handover demo node.
Requires the full stack to already be running:

  ros2 launch dual_arm bringup.launch.py use_fake_hardware:=true

The running move_group already has ExecuteTaskSolutionCapability loaded
(set in moveit_config.py), so MTC can submit trajectories through it.
"""

import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from moveit_config import declare_launch_args, get_moveit_params


def generate_launch_description():
    launch_args, lc = declare_launch_args()
    moveit_params = get_moveit_params(lc)

    mtc_demo_node = Node(
        package="dual_arm",
        executable="mtc_handover",
        output="screen",
        parameters=moveit_params + [
            {"use_fake_hardware": lc["use_fake_hardware"]},
            # MTC-specific overrides
            {"trajectory_execution.allowed_start_tolerance": 0.01},
            {"ompl": {"start_state_max_bounds_error": 0.05}},
        ],
    )

    return LaunchDescription(
        launch_args + [
            TimerAction(period=2.0, actions=[mtc_demo_node]),
        ]
    )
