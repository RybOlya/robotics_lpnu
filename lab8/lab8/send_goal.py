#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .limits import JOINT_ORDER, SAFE_PRESETS, validate_joint_goal


class GoalPublisher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("lab8_goal_sender")
        self.pub = self.create_publisher(JointState, topic, 10)
        self.topic = topic

    def send(self, goal) -> None:
        msg = JointState()
        msg.name = JOINT_ORDER
        msg.position = [float(v) for v in goal]
        for _ in range(3):
            self.pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info(f"Sent joint goal on {self.topic}: {msg.position}")


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Send validated SO-101 joint goal")
    parser.add_argument("--topic", default="/arm_goal_ticks", help="Goal topic")
    parser.add_argument(
        "--preset",
        choices=sorted(SAFE_PRESETS.keys()),
        default="ready",
        help="Safe preset base pose",
    )
    parser.add_argument(
        "--joints",
        nargs=6,
        type=float,
        metavar=("PAN", "LIFT", "ELBOW", "WFLEX", "WROLL", "GRIP"),
        help="Full 6-joint goal in radians (+ gripper 0..1)",
    )
    for joint_name in JOINT_ORDER:
        parser.add_argument(f"--{joint_name}", type=float, help=f"Override {joint_name}")
    return parser.parse_args(argv)


def _build_goal(args):
    if args.joints is not None:
        goal = list(args.joints)
    else:
        goal = list(SAFE_PRESETS[args.preset])
    for idx, joint_name in enumerate(JOINT_ORDER):
        value = getattr(args, joint_name)
        if value is not None:
            goal[idx] = float(value)
    return goal


def main(args=None) -> None:
    user_args = _parse_args(sys.argv[1:] if args is None else args)
    goal = _build_goal(user_args)
    ok, reason = validate_joint_goal(goal)
    if not ok:
        print(f"Rejected goal: {reason}")
        sys.exit(2)

    rclpy.init(args=None)
    node = GoalPublisher(user_args.topic)
    time.sleep(0.3)
    node.send(goal)
    node.destroy_node()
    rclpy.shutdown()

