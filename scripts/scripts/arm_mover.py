"""
arm_mover.py

MoveItPy based Robot-Arm Control Wrapper.

Planning groups (from SRDF):
  - "left_fr3_arm"   : left arm only  (7 joints), EE = left_fr3_hand_tcp
  - "right_fr3_arm"  : right arm only (7 joints), EE = right_fr3_hand_tcp
  - "both_arms"      : both arms      (14 joints, left j1-j7 first then right j1-j7)
                       use move_to_joint_positions([7 left values, 7 right values])
                       pose goals are NOT supported for both_arms
"""

import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped


class ArmMover:
    """
    Parameters
    ----------
    node_name   : str  - ROS 2 node name
    group_name  : str  - Planning group defined in SRDF
                         "left_fr3_arm", "right_fr3_arm", or "both_arms"
    ee_link     : str  - End-effector link for pose goals
                         "left_fr3_hand_tcp" or "right_fr3_hand_tcp"
                         (ignored when using both_arms)
    base_frame  : str  - Reference frame for pose goals, e.g. "world"
    """

    def __init__(
        self,
        node_name: str = "arm_mover",
        group_name: str = "left_fr3_arm",
        ee_link: str = "left_fr3_hand_tcp",
        base_frame: str = "world",
    ):
        self.logger = get_logger(node_name)
        self.group_name = group_name
        self.ee_link = ee_link
        self.base_frame = base_frame

        self.moveit = MoveItPy(node_name=node_name)
        self.arm = self.moveit.get_planning_component(group_name)
        self.robot_model = self.moveit.get_robot_model()

        self.logger.info(
            f"ArmMover ready | group={group_name} | ee={ee_link} | frame={base_frame}"
        )

    # ------------------------------------------------------------------
    # Pose goal  (single-arm groups only: left_fr3_arm / right_fr3_arm)
    # ------------------------------------------------------------------
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
        velocity_scale: float = 0.5,
        planning_time: float = 5.0,
    ) -> bool:
        """
        Move end-effector to the specified pose.
        Returns True on success, False on failure.
        """
        goal = PoseStamped()
        goal.header.frame_id = self.base_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=goal,
            pose_link=self.ee_link,
        )

        return self._plan_and_execute(velocity_scale, planning_time)

    # ------------------------------------------------------------------
    # Joint goal  (all groups supported)
    # ------------------------------------------------------------------
    def move_to_joint_positions(
        self,
        positions: list,
        velocity_scale: float = 0.5,
        planning_time: float = 5.0,
    ) -> bool:
        """
        Move joints to the specified angles (radians).

        Parameters
        ----------
        positions : list[float]
            7 values for left_fr3_arm or right_fr3_arm.
            14 values for both_arms (left j1-j7 first, then right j1-j7).
        """
        goal_state = RobotState(self.robot_model)
        goal_state.set_joint_group_positions(self.group_name, positions)
        goal_state.update()

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=goal_state)

        return self._plan_and_execute(velocity_scale, planning_time)

    # ------------------------------------------------------------------
    # Named target  (SRDF states: "ready", "extended", "open", "close")
    # ------------------------------------------------------------------
    def move_to_named(
        self,
        named_target: str,
        velocity_scale: float = 0.5,
        planning_time: float = 5.0,
    ) -> bool:
        """
        Move to a named state defined in the SRDF.
        Available: "ready", "extended" (arm groups); "open", "close" (hand groups).
        """
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=named_target)

        return self._plan_and_execute(velocity_scale, planning_time)

    # ------------------------------------------------------------------
    # Internal: plan → execute
    # ------------------------------------------------------------------
    def _plan_and_execute(
        self,
        velocity_scale: float,
        planning_time: float,
    ) -> bool:
        plan_result = self.arm.plan(
            single_plan_parameters={
                "max_velocity_scaling_factor": velocity_scale,
                "allowed_planning_time": planning_time,
            }
        )

        if not plan_result:
            self.logger.error("Planning failed")
            return False

        self.logger.info("Planning succeeded, executing...")
        self.moveit.execute(plan_result.trajectory, controllers=[])
        self.logger.info("Execution complete")
        return True
