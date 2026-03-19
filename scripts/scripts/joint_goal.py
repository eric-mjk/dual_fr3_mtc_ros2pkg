#!/usr/bin/env python3
"""
joint_goal.py
-------------
Move arm(s) to joint positions or a named SRDF state.
Requires move_group node to be running (launch the stack first).

Usage:
    ros2 run scripts joint_goal
    ros2 run scripts joint_goal --named ready
    ros2 run scripts joint_goal --joints 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785

For both arms (14 joint values, left arm first then right arm):
    ros2 run scripts joint_goal \
        --group both_arms \
        --joints 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785 \
                 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785

Named states (SRDF): "ready", "extended"
"""

import sys
import argparse
import rclpy
from scripts.arm_mover import ArmMover

# FR3 "ready" pose from SRDF: j1=0, j2=-π/4, j3=0, j4=-3π/4, j5=0, j6=π/2, j7=π/4
_READY_POSE = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def parse_args():
    parser = argparse.ArgumentParser(description="Move arm to joint positions or named target")
    parser.add_argument("--group",  default="left_fr3_arm",      help="Planning group name (SRDF)")
    parser.add_argument("--ee",     default="left_fr3_hand_tcp",  help="End-effector link name (URDF)")
    parser.add_argument("--frame",  default="world",               help="Reference frame")
    parser.add_argument("--joints", nargs="+", type=float,
                        default=None,
                        help="Joint positions in radians (7 values for one arm, 14 for both_arms)")
    parser.add_argument("--named",  default=None,
                        help="Named target from SRDF: ready, extended")
    parser.add_argument("--vel",    type=float, default=0.5,  help="Velocity scale (0-1)")
    parser.add_argument("--time",   type=float, default=5.0,  help="Max planning time (seconds)")

    return parser.parse_args(args=[a for a in sys.argv[1:] if not a.startswith("__")])


def main():
    rclpy.init()
    args = parse_args()

    mover = ArmMover(
        node_name="joint_goal_node",
        group_name=args.group,
        ee_link=args.ee,
        base_frame=args.frame,
    )

    if args.named and args.named.strip():
        print(f"\nNamed target: '{args.named}'\n")
        success = mover.move_to_named(
            named_target=args.named,
            velocity_scale=args.vel,
            planning_time=args.time,
        )

    elif args.joints and any(args.joints):
        print(f"\nJoint positions (rad): {args.joints}\n")
        success = mover.move_to_joint_positions(
            positions=args.joints,
            velocity_scale=args.vel,
            planning_time=args.time,
        )

    else:
        # Default: "ready" pose (matches SRDF named state)
        default_joints = _READY_POSE
        print(f"\nDefault joint positions — ready pose (rad): {default_joints}\n")
        success = mover.move_to_joint_positions(
            positions=default_joints,
            velocity_scale=args.vel,
            planning_time=args.time,
        )

    if success:
        print("SUCCESS: reached target joint positions.")
    else:
        print("FAILED: planning failed. Check group name / joint count.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
