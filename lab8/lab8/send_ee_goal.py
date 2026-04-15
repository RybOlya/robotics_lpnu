#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class EeGoalPublisher(Node):
    def __init__(self, topic: str, frame_id: str) -> None:
        super().__init__("lab8_ee_goal_sender")
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.topic = topic
        self.frame_id = frame_id

    def send(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        for _ in range(3):
            self.pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info(f"Published EE goal on {self.topic}")


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Publish desired end-effector pose")
    parser.add_argument("--topic", default="/ee_goal", help="EE goal topic")
    parser.add_argument("--frame", default="base", help="Frame id")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    return parser.parse_args(argv)


def main(args=None) -> None:
    user_args = _parse_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=None)
    node = EeGoalPublisher(user_args.topic, user_args.frame)
    time.sleep(0.3)
    node.send(
        user_args.x,
        user_args.y,
        user_args.z,
        user_args.roll,
        user_args.pitch,
        user_args.yaw,
    )
    node.destroy_node()
    rclpy.shutdown()

