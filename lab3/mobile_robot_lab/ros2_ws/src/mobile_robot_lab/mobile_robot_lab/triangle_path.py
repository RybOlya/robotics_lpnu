import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .diff_drive_math import twist_to_wheel_speeds

class TimedMotion(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.160)

    def publish_for(self, v: float, w: float, duration_s: float):
        rate_hz = float(self.get_parameter("rate_hz").value)
        dt = 1.0 / max(rate_hz, 1.0)

        wheel_radius = float(self.get_parameter("wheel_radius").value)
        wheel_sep = float(self.get_parameter("wheel_separation").value)
        wl, wr = twist_to_wheel_speeds(v, w, wheel_radius, wheel_sep)

        self.get_logger().info(
            f"Segment: v={v:.2f}, w={w:.2f}, t={duration_s:.2f}s | wheel ω: L={wl:.2f}, R={wr:.2f}"
        )

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w

        t_end = time.time() + max(duration_s, 0.0)
        while time.time() < t_end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def stop(self):
        self.pub.publish(Twist())
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TimedMotion("triangle_path")

    node.declare_parameter("side_length", 2.0)     # meters
    node.declare_parameter("linear_speed", 0.20)   # m/s
    node.declare_parameter("turn_speed", 0.60)     # rad/s

    side = float(node.get_parameter("side_length").value)
    v = float(node.get_parameter("linear_speed").value)
    w = float(node.get_parameter("turn_speed").value)

    # Equilateral triangle: external turn angle = 120° = 2*pi/3
    forward_time = side / max(v, 1e-6)
    turn_time = (2.0 * math.pi / 3.0) / max(w, 1e-6)

    node.get_logger().info("Executing triangle path (3 sides + 3 turns)...")

    for i in range(3):
        node.get_logger().info(f"Side {i+1}/3")
        node.publish_for(v, 0.0, forward_time)
        node.publish_for(0.0, w, turn_time)

    node.stop()
    node.get_logger().info("Triangle complete.")
    node.destroy_node()
    rclpy.shutdown()
