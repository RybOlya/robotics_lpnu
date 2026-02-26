import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .diff_drive_math import twist_to_wheel_speeds, curve_radius

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")

        # Parameters (students can change via --ros-args -p ...)
        self.declare_parameter("linear_x", 0.20)
        self.declare_parameter("angular_z", 0.00)
        self.declare_parameter("rate_hz", 10.0)

        # Diff-drive parameters (approximate TurtleBot3 Burger defaults)
        self.declare_parameter("wheel_radius", 0.033)       # meters
        self.declare_parameter("wheel_separation", 0.160)   # meters

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        period = 1.0 / max(rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info("velocity_publisher started. Publishing to /cmd_vel")

    def on_timer(self):
        v = float(self.get_parameter("linear_x").value)
        w = float(self.get_parameter("angular_z").value)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

        r = curve_radius(v, w)
        wheel_radius = float(self.get_parameter("wheel_radius").value)
        wheel_sep = float(self.get_parameter("wheel_separation").value)
        wl, wr = twist_to_wheel_speeds(v, w, wheel_radius, wheel_sep)

        r_txt = "inf" if r == float("inf") else f"{r:.3f} m"
        self.get_logger().info(
            f"cmd_vel: v={v:.2f} m/s, w={w:.2f} rad/s | radius={r_txt} | wheel ω: L={wl:.2f} rad/s, R={wr:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
