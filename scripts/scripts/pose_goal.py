#!/usr/bin/env python3
"""
pose_goal.py
------------
Move a single arm's end-effector to a target pose.
Requires move_group node to be running (launch the stack first).

Usage:
    ros2 run scripts pose_goal

With arguments:
    ros2 run scripts pose_goal \
        --group left_fr3_arm \
        --ee left_fr3_hand_tcp \
        --frame world \
        --pos 0.4 0.0 0.5 \
        --quat 0.0 0.0 0.0 1.0

Groups:  left_fr3_arm | right_fr3_arm
         (both_arms not supported for pose goals — use joint_goal instead)
"""

import sys
import argparse
import rclpy
from scripts.arm_mover import ArmMover


def parse_args():
    parser = argparse.ArgumentParser(description="Move arm to a target pose")
    parser.add_argument("--group",  default="left_fr3_arm",     help="Planning group name (SRDF)")
    parser.add_argument("--ee",     default="left_fr3_hand_tcp", help="End-effector link name (URDF)")
    parser.add_argument("--frame",  default="world",              help="Reference frame")
    parser.add_argument("--pos",    nargs=3, type=float,
                        default=[0.4, 0.0, 0.5],
                        metavar=("X", "Y", "Z"),
                        help="Target position (meters)")
    parser.add_argument("--quat",   nargs=4, type=float,
                        default=[0.0, 0.0, 0.0, 1.0],
                        metavar=("QX", "QY", "QZ", "QW"),
                        help="Target orientation as quaternion")
    parser.add_argument("--vel",    type=float, default=0.5,  help="Velocity scale (0-1)")
    parser.add_argument("--time",   type=float, default=5.0,  help="Max planning time (seconds)")

    return parser.parse_args(args=[a for a in sys.argv[1:] if not a.startswith("__")])


def main():
    rclpy.init()
    args = parse_args()

    mover = ArmMover(
        node_name="pose_goal_node",
        group_name=args.group,
        ee_link=args.ee,
        base_frame=args.frame,
    )

    x, y, z = args.pos
    qx, qy, qz, qw = args.quat

    print(f"\nTarget pose:")
    print(f"  position    : x={x}, y={y}, z={z}")
    print(f"  orientation : qx={qx}, qy={qy}, qz={qz}, qw={qw}")
    print(f"  frame       : {args.frame}\n")

    success = mover.move_to_pose(
        x=x, y=y, z=z,
        qx=qx, qy=qy, qz=qz, qw=qw,
        velocity_scale=args.vel,
        planning_time=args.time,
    )

    if success:
        print("SUCCESS: reached target pose.")
    else:
        print("FAILED: planning failed. Check group name / ee link / frame.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
