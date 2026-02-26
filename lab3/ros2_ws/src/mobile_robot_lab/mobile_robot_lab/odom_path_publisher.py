import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomPathPublisher(Node):
    """
    Subscribes to /odom and publishes a growing nav_msgs/Path on /path,
    which can be displayed directly in RViz2.
    """
    def __init__(self):
        super().__init__("odom_path_publisher")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("max_poses", 2000)

        odom_topic = self.get_parameter("odom_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.max_poses = int(self.get_parameter("max_poses").value)

        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Path, self.path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.get_logger().info(f"Listening on {odom_topic}, publishing Path on {self.path_topic}")

    def on_odom(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Force desired frame_id for RViz consistency
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.header.frame_id = self.frame_id
        pose.header.frame_id = self.frame_id

        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses = self.path_msg.poses[-self.max_poses:]

        self.pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
