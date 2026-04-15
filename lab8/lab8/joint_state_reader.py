#!/usr/bin/env python3

import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from visualization_msgs.msg import Marker
import yaml

from .limits import JOINT_LIMITS, validate_joint_goal
from .kinematics import fk_pose, ik_position


class JointStateReader(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_reader")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 1000000)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_twist", True)
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/robot/cmd_pose")
        self.declare_parameter("goal_topic", "/arm_goal_ticks")
        self.declare_parameter("ee_goal_topic", "/ee_goal")
        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("ee_marker_topic", "/ee_marker")
        self.declare_parameter("calibration_file", "")

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.publish_twist = self.get_parameter("publish_twist").get_parameter_value().bool_value
        self.joint_topic = self.get_parameter("joint_topic").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self.ee_goal_topic = self.get_parameter("ee_goal_topic").get_parameter_value().string_value
        self.ee_pose_topic = self.get_parameter("ee_pose_topic").get_parameter_value().string_value
        self.ee_marker_topic = self.get_parameter("ee_marker_topic").get_parameter_value().string_value
        self.calibration_file = (
            self.get_parameter("calibration_file").get_parameter_value().string_value
        )

        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]
        self.servo_ids = [1, 2, 3, 4, 5, 6]
        self.last_positions = [0.0] * len(self.joint_names)
        self.calibration_by_joint = self._load_calibration()

        self.joint_pub = self.create_publisher(JointState, self.joint_topic, 10)
        self.twist_pub = self.create_publisher(Twist, self.pose_topic, 10)
        self.goal_sub = self.create_subscription(
            JointState, self.goal_topic, self._on_goal, 10
        )
        self.ee_goal_sub = self.create_subscription(
            PoseStamped, self.ee_goal_topic, self._on_ee_goal, 10
        )
        self.ee_pose_pub = self.create_publisher(PoseStamped, self.ee_pose_topic, 10)
        self.ee_marker_pub = self.create_publisher(Marker, self.ee_marker_topic, 10)
        self.serial_port = None
        self._connect()
        self.timer = self.create_timer(0.05, self._tick)

    def _connect(self) -> None:
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            time.sleep(0.1)
            self.get_logger().info(
                f"Connected to {self.port} at {self.baud_rate} baud"
            )
        except Exception as exc:
            self.serial_port = None
            self.get_logger().error(f"Serial open failed: {exc}")

    def _read_servo_pos(self, servo_id: int):
        if self.serial_port is None:
            return None
        try:
            length = 4
            instruction = 0x02
            address = 0x38
            read_length = 0x02
            checksum = (~(servo_id + length + instruction + address + read_length)) & 0xFF
            cmd = bytes(
                [0xFF, 0xFF, servo_id, length, instruction, address, read_length, checksum]
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.write(cmd)
            time.sleep(0.002)
            resp = self.serial_port.read(8)
            if len(resp) < 7:
                return None
            if resp[0] != 0xFF or resp[1] != 0xFF or resp[2] != servo_id:
                return None
            return struct.unpack("<H", resp[5:7])[0]
        except Exception:
            return None

    def _load_calibration(self):
        defaults = {
            joint_name: {"range_min": 0, "range_max": 4095}
            for joint_name in self.joint_names
        }
        if not self.calibration_file:
            self.get_logger().warning("No calibration_file set, using default raw ranges")
            return defaults
        try:
            with open(self.calibration_file, "r", encoding="utf-8") as infile:
                data = yaml.safe_load(infile) or {}
            for joint_name in self.joint_names:
                joint_data = data.get(joint_name, {})
                range_min = int(joint_data.get("range_min", defaults[joint_name]["range_min"]))
                range_max = int(joint_data.get("range_max", defaults[joint_name]["range_max"]))
                if range_max <= range_min:
                    raise ValueError(f"{joint_name} has invalid range [{range_min}, {range_max}]")
                defaults[joint_name] = {"range_min": range_min, "range_max": range_max}
            self.get_logger().info(f"Loaded calibration from {self.calibration_file}")
            return defaults
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to load calibration file '{self.calibration_file}': {exc}"
            )
            self.get_logger().warning("Using default raw ranges")
            return defaults

    def _ticks_to_joint(self, joint_name: str, ticks: int) -> float:
        calib = self.calibration_by_joint[joint_name]
        range_min = float(calib["range_min"])
        range_max = float(calib["range_max"])
        lower, upper = JOINT_LIMITS[joint_name]
        progress = (float(ticks) - range_min) / (range_max - range_min)
        progress = max(0.0, min(1.0, progress))
        return lower + progress * (upper - lower)

    def _joint_to_ticks(self, joint_name: str, value: float) -> int:
        calib = self.calibration_by_joint[joint_name]
        range_min = float(calib["range_min"])
        range_max = float(calib["range_max"])
        lower, upper = JOINT_LIMITS[joint_name]
        v = max(lower, min(upper, float(value)))
        progress = (v - lower) / (upper - lower) if upper > lower else 0.0
        ticks = int(round(range_min + progress * (range_max - range_min)))
        return max(0, min(4095, ticks))

    def _write_goal(self, servo_id: int, ticks: int, speed: int = 2400, acc: int = 50) -> bool:
        if self.serial_port is None:
            return False
        try:
            ticks = max(0, min(4095, int(ticks)))
            speed = max(0, min(4095, int(speed)))
            acc = max(0, min(255, int(acc)))
            mem_addr = 41
            instruction = 0x03
            params = [
                mem_addr,
                acc & 0xFF,
                ticks & 0xFF,
                (ticks >> 8) & 0xFF,
                0,
                0,
                speed & 0xFF,
                (speed >> 8) & 0xFF,
            ]
            length = len(params) + 2
            checksum = (~(servo_id + length + instruction + sum(params))) & 0xFF
            packet = bytes([0xFF, 0xFF, servo_id, length, instruction] + params + [checksum])
            self.serial_port.write(packet)
            return True
        except Exception:
            return False

    def _on_goal(self, msg: JointState) -> None:
        if self.serial_port is None:
            return
        if len(msg.position) < len(self.servo_ids):
            self.get_logger().warning(
                f"Goal ignored: expected {len(self.servo_ids)} values, got {len(msg.position)}"
            )
            return
        goal = [float(msg.position[i]) for i in range(len(self.servo_ids))]
        is_valid, reason = validate_joint_goal(goal)
        if not is_valid:
            self.get_logger().warning(f"Goal rejected: {reason}")
            return
        for index, servo_id in enumerate(self.servo_ids):
            joint_name = self.joint_names[index]
            ticks = self._joint_to_ticks(joint_name, goal[index])
            self._write_goal(servo_id, ticks)

    def _on_ee_goal(self, msg: PoseStamped) -> None:
        if self.serial_port is None:
            return
        current_gripper = self.last_positions[5] if len(self.last_positions) >= 6 else 0.5
        ok, reason, goal = ik_position(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            current_gripper=current_gripper,
        )
        if not ok:
            self.get_logger().warning(f"EE goal rejected: {reason}")
            return
        for index, servo_id in enumerate(self.servo_ids):
            joint_name = self.joint_names[index]
            ticks = self._joint_to_ticks(joint_name, goal[index])
            self._write_goal(servo_id, ticks)
        self.get_logger().info(
            f"EE goal -> joint goal: {[round(v, 3) for v in goal]}"
        )

    def _publish_ee(self, positions) -> None:
        fk = fk_pose(positions)
        now = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = fk["x"]
        pose_msg.pose.position.y = fk["y"]
        pose_msg.pose.position.z = fk["z"]
        pose_msg.pose.orientation.x = fk["qx"]
        pose_msg.pose.orientation.y = fk["qy"]
        pose_msg.pose.orientation.z = fk["qz"]
        pose_msg.pose.orientation.w = fk["qw"]
        self.ee_pose_pub.publish(pose_msg)

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.frame_id
        marker.ns = "ee"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = fk["x"]
        marker.pose.position.y = fk["y"]
        marker.pose.position.z = fk["z"]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        self.ee_marker_pub.publish(marker)

    def _tick(self) -> None:
        if self.serial_port is None:
            self._connect()
            return

        positions = []
        valid = 0
        for idx, servo_id in enumerate(self.servo_ids):
            ticks = self._read_servo_pos(servo_id)
            if ticks is None:
                positions.append(self.last_positions[idx])
                continue
            valid += 1
            joint_name = self.joint_names[idx]
            pos = self._ticks_to_joint(joint_name, ticks)
            positions.append(pos)

        if valid == 0:
            return

        now = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        self.joint_pub.publish(msg)
        self._publish_ee(positions)

        if self.publish_twist:
            twist = Twist()
            twist.angular.z = positions[0] - self.last_positions[0]
            twist.angular.y = positions[1] - self.last_positions[1]
            twist.angular.x = positions[2] - self.last_positions[2]
            twist.linear.y = positions[3] - self.last_positions[3]
            twist.linear.x = positions[4] - self.last_positions[4]
            twist.linear.z = positions[5] - self.last_positions[5]
            self.twist_pub.publish(twist)

        self.last_positions = positions


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.serial_port is not None:
        node.serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

