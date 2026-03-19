"""
bringup.launch.py
-----------------
Drop-in replacement for refactor_moveit.launch.py that pulls all
MoveIt config from moveit_config.py (single source of truth).

Usage (identical to refactor_moveit.launch.py):
  ros2 launch dual_arm bringup.launch.py use_fake_hardware:=true
  ros2 launch dual_arm bringup.launch.py use_fake_hardware:=true load_gripper:=false
  ros2 launch dual_arm bringup.launch.py \
      left_robot_ip:=<IP> right_robot_ip:=<IP> use_fake_hardware:=false
"""

import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_config import (
    declare_launch_args,
    get_moveit_params,
    get_robot_description,
    get_robot_description_semantic,
    get_planning_pipelines,
    get_ros2_control_node,
    load_yaml,
)


def generate_launch_description():
    # ------------------------------------------------------------------
    # Shared hardware args + MoveIt config
    # ------------------------------------------------------------------
    launch_args, lc = declare_launch_args()
    moveit_params = get_moveit_params(lc)

    robot_description          = get_robot_description(lc)
    robot_description_semantic = get_robot_description_semantic(lc)
    planning_pipelines         = get_planning_pipelines()
    kinematics_yaml            = load_yaml("dual_arm", "config/dual_kinematics.yaml") or {}

    dual_arm_share = get_package_share_directory("dual_arm")

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_params,
        arguments=["--ros-args", "--param", "publish_robot_description_semantic:=true"],
    )

    rviz_config = os.path.join(dual_arm_share, "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines,
            kinematics_yaml,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Selects dual_ros2_controllers_fake.yaml (position) or
    # dual_ros2_controllers.yaml (effort) based on use_fake_hardware
    ros2_control_nodes = get_ros2_control_node(lc)

    load_controllers = [
        ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", controller,
                 "--controller-manager-timeout", "60",
                 "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for controller in ["joint_state_broadcaster", "left_arm_controller", "right_arm_controller"]
    ]

    joint_state_aggregator = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_aggregator",
        output="screen",
        parameters=[robot_description, {
            "source_list": [
                "/arm/joint_states",
                "/left/franka_gripper/joint_states",
                "/right/franka_gripper/joint_states",
            ],
            "rate": 50.0,
        }],
    )

    franka_robot_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "franka_robot_state_broadcaster",
            "--controller-manager-timeout", "60",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(lc["use_fake_hardware"]),
    )

    left_gripper_fake = Node(
        package='dual_arm',
        executable='fake_gripper_action_server.py',
        name='franka_gripper',
        namespace='left',
        parameters=[{'joint_names': ['left_fr3_finger_joint1', 'left_fr3_finger_joint2']}],
        output='screen',
        condition=IfCondition(lc["use_fake_hardware"]),
    )
    left_gripper_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
        ),
        launch_arguments={
            'robot_ip': lc["left_robot_ip"],
            'use_fake_hardware': 'false',
            'namespace': 'left',
            'arm_id': 'left_fr3',
        }.items(),
        condition=UnlessCondition(lc["use_fake_hardware"]),
    )

    right_gripper_fake = Node(
        package='dual_arm',
        executable='fake_gripper_action_server.py',
        name='franka_gripper',
        namespace='right',
        parameters=[{'joint_names': ['right_fr3_finger_joint1', 'right_fr3_finger_joint2']}],
        output='screen',
        condition=IfCondition(lc["use_fake_hardware"]),
    )
    right_gripper_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
        ),
        launch_arguments={
            'robot_ip': lc["right_robot_ip"],
            'use_fake_hardware': 'false',
            'namespace': 'right',
            'arm_id': 'right_fr3',
        }.items(),
        condition=UnlessCondition(lc["use_fake_hardware"]),
    )

    left_gripper_group  = GroupAction([left_gripper_fake,  left_gripper_real],  condition=IfCondition(lc["load_gripper"]))
    right_gripper_group = GroupAction([right_gripper_fake, right_gripper_real], condition=IfCondition(lc["load_gripper"]))

    return LaunchDescription(
        launch_args + [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            joint_state_aggregator,
            franka_robot_state_broadcaster,
            left_gripper_group,
            right_gripper_group,
        ] + ros2_control_nodes + load_controllers
    )
