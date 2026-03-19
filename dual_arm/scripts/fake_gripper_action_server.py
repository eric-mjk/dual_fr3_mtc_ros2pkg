#!/usr/bin/env python3
import rclpy
from rclpy import Parameter
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand


class FakeGripperActionServer(Node):

    def __init__(self):
        super().__init__('franka_gripper')

        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.joint_names = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        assert len(self.joint_names) == 2

        self._position = 0.035  # per finger, start open

        self._publisher = self.create_publisher(JointState, '~/joint_states', 1)
        self._timer = self.create_timer(0.1, self._publish_state)

        self._action_server = ActionServer(
            self,
            GripperCommand,
            '~/gripper_action',
            execute_callback=self._execute_callback,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            f'FakeGripperActionServer ready. Joints: {self.joint_names}'
        )

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [self._position, self._position]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self._publisher.publish(msg)

    def _execute_callback(self, goal_handle):
        # MoveIt sends total gap width (sum of both fingers) — divide by 2 per finger
        target = goal_handle.request.command.position / 2.0
        target = max(0.0, min(0.04, target))  # clamp to joint limits [0, 0.04]

        self._position = target

        feedback = GripperCommand.Feedback()
        feedback.position = target * 2.0
        feedback.effort = 0.0
        feedback.stalled = False
        feedback.reached_goal = False
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = GripperCommand.Result()
        result.position = target * 2.0
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeGripperActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
