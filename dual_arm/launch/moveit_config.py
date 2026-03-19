"""
moveit_config.py
----------------
Shared MoveIt configuration builder for the dual_arm package.

Both refactor_moveit.launch.py and run_script.launch.py import this
module so that config changes only need to be made in one place.

Usage in a launch file:
    from launch.moveit_config import get_moveit_params, declare_launch_args, load_yaml

    def generate_launch_description():
        launch_args, lc = declare_launch_args()
        moveit_params = get_moveit_params(lc)
        ...
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name: str, file_path: str) -> dict:
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def declare_launch_args() -> tuple:
    """
    Declare all hardware / robot args shared across launch files.
    Returns (list of DeclareLaunchArgument, dict of LaunchConfiguration).
    """
    args = {
        "left_robot_ip":        ("dont-care",   "IP of the left FR3"),
        "right_robot_ip":       ("dont-care",   "IP of the right FR3"),
        "use_fake_hardware":    ("true",         "Use fake hardware (simulation)"),
        "fake_sensor_commands": ("true",         "Enable fake sensor commands"),
        "load_gripper":         ("true",         "Load gripper hardware/fake publisher"),
        "ee_id":                ("franka_hand",  "End-effector ID"),
        "left_xyz":             ("0 -0.3 0",     "Left arm base position xyz"),
        "left_rpy":             ("0 0 0",        "Left arm base orientation rpy"),
        "right_xyz":            ("0 0.3 0",      "Right arm base position xyz"),
        "right_rpy":            ("0 0 0",        "Right arm base orientation rpy"),
    }

    declare = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, (default, desc) in args.items()
    ]
    lc = {name: LaunchConfiguration(name) for name in args}

    return declare, lc


def get_robot_description(lc: dict) -> dict:
    """Build robot_description param from xacro (URDF)."""
    dual_arm_share = get_package_share_directory("dual_arm")
    urdf_xacro = os.path.join(dual_arm_share, "config", "dual_fr3.urdf.xacro")

    robot_description_config = Command([
        FindExecutable(name="xacro"), " ", urdf_xacro,
        " hand:=",                lc["load_gripper"],
        " ee_id:=",               lc["ee_id"],
        " left_robot_ip:=",       lc["left_robot_ip"],
        " right_robot_ip:=",      lc["right_robot_ip"],
        " use_fake_hardware:=",   lc["use_fake_hardware"],
        " fake_sensor_commands:=",lc["fake_sensor_commands"],
        " ros2_control:=true",
        " left_xyz:=\"",  lc["left_xyz"],  "\"",
        " left_rpy:=\"",  lc["left_rpy"],  "\"",
        " right_xyz:=\"", lc["right_xyz"], "\"",
        " right_rpy:=\"", lc["right_rpy"], "\"",
    ])
    return {"robot_description": ParameterValue(robot_description_config, value_type=str)}


def get_robot_description_semantic(lc: dict) -> dict:
    """Build robot_description_semantic param from xacro (SRDF)."""
    dual_arm_share = get_package_share_directory("dual_arm")
    srdf_xacro = os.path.join(dual_arm_share, "config", "dual_fr3.srdf.xacro")

    robot_description_semantic_config = Command([
        FindExecutable(name="xacro"), " ", srdf_xacro,
        " hand:=", lc["load_gripper"],
        " ee_id:=", lc["ee_id"],
    ])
    return {"robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)}


def get_planning_pipelines() -> dict:
    """Build OMPL planning pipeline config."""
    ompl_yaml = load_yaml("dual_arm", "config/ompl_planning.yaml") or {}
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/ResolveConstraintFrames "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    planning_pipelines["ompl"].update(ompl_yaml)
    return planning_pipelines


def get_move_group_capabilities() -> dict:
    """
    move_group capabilities.
    ExecuteTaskSolutionCapability is always included so MTC works
    regardless of which launch file started move_group.
    """
    return {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
    }


def get_moveit_controllers() -> dict:
    """Build MoveItSimpleControllerManager config."""
    moveit_simple_controllers_yaml = load_yaml("dual_arm", "config/dual_fr3_controllers.yaml") or {}
    return {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
    }


def get_trajectory_execution() -> dict:
    return {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }


def get_planning_scene_monitor() -> dict:
    return {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


def get_ros2_control_node(lc: dict) -> list:
    """
    Return a list of two ros2_control_node definitions — one for fake hardware
    (position command interface) and one for real hardware (effort command interface).
    Exactly one will be active at launch time based on use_fake_hardware.
    """
    dual_arm_share = get_package_share_directory("dual_arm")
    fake_controllers = os.path.join(dual_arm_share, "config", "dual_ros2_controllers_fake.yaml")
    real_controllers = os.path.join(dual_arm_share, "config", "dual_ros2_controllers.yaml")

    common_remappings = [
        ("~/robot_description", "/robot_description"),
        ("joint_states", "/arm/joint_states"),
    ]

    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output={"stdout": "screen", "stderr": "screen"},
            parameters=[fake_controllers],
            remappings=common_remappings,
            on_exit=Shutdown(),
            condition=IfCondition(lc["use_fake_hardware"]),
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output={"stdout": "screen", "stderr": "screen"},
            parameters=[real_controllers],
            remappings=common_remappings,
            on_exit=Shutdown(),
            condition=UnlessCondition(lc["use_fake_hardware"]),
        ),
    ]


def get_moveit_params(lc: dict) -> list:
    """
    Return a list of parameter dicts to pass to a Node's parameters=[].
    Includes everything needed by both move_group and MoveItPy.
    """
    kinematics_yaml = load_yaml("dual_arm", "config/dual_kinematics.yaml") or {}
    return [
        get_robot_description(lc),
        get_robot_description_semantic(lc),
        kinematics_yaml,
        get_planning_pipelines(),
        get_move_group_capabilities(),
        get_moveit_controllers(),
        get_trajectory_execution(),
        get_planning_scene_monitor(),
    ]
