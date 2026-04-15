#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from .limits import JOINT_ORDER, validate_joint_goal


class MotionRunner(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("lab8_motion_runner")
        self.pub = self.create_publisher(JointState, topic, 10)
        self.topic = topic

    def publish_goal(self, goal) -> None:
        msg = JointState()
        msg.name = JOINT_ORDER
        msg.position = [float(v) for v in goal]
        for _ in range(3):
            self.pub.publish(msg)
            time.sleep(0.05)


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Run joint waypoint motion from YAML")
    parser.add_argument("--file", required=True, help="YAML file with waypoints")
    parser.add_argument("--topic", default="/arm_goal_ticks", help="Goal topic")
    return parser.parse_args(argv)


def _load_waypoints(path):
    with open(path, "r", encoding="utf-8") as infile:
        data = yaml.safe_load(infile) or {}
    waypoints = data.get("waypoints", [])
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("YAML must contain non-empty 'waypoints' list")
    parsed = []
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict) or "joints" not in waypoint:
            raise ValueError(f"Waypoint {idx} missing 'joints'")
        goal = [float(v) for v in waypoint["joints"]]
        ok, reason = validate_joint_goal(goal)
        if not ok:
            raise ValueError(f"Waypoint {idx} rejected: {reason}")
        hold_sec = float(waypoint.get("hold_sec", 1.0))
        parsed.append({"name": waypoint.get("name", f"wp_{idx}"), "joints": goal, "hold_sec": hold_sec})
    return parsed


def main(args=None) -> None:
    user_args = _parse_args(sys.argv[1:] if args is None else args)
    waypoints = _load_waypoints(user_args.file)
    rclpy.init(args=None)
    node = MotionRunner(user_args.topic)
    time.sleep(0.3)
    for waypoint in waypoints:
        node.publish_goal(waypoint["joints"])
        node.get_logger().info(f"Waypoint {waypoint['name']} sent")
        time.sleep(max(0.0, waypoint["hold_sec"]))
    node.destroy_node()
    rclpy.shutdown()

