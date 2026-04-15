import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_new_calib.urdf",
    )
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    port = LaunchConfiguration("port")
    baud_rate = LaunchConfiguration("baud_rate")
    frame_id = LaunchConfiguration("frame_id")
    publish_twist = LaunchConfiguration("publish_twist")
    joint_topic = LaunchConfiguration("joint_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    ee_goal_topic = LaunchConfiguration("ee_goal_topic")
    ee_pose_topic = LaunchConfiguration("ee_pose_topic")
    ee_marker_topic = LaunchConfiguration("ee_marker_topic")
    calibration_file = LaunchConfiguration("calibration_file")
    rviz_config = LaunchConfiguration("rviz_config")

    joint_state_reader = Node(
        package="lab8",
        executable="joint_state_reader",
        name="joint_state_reader",
        output="screen",
        parameters=[
            {
                "port": port,
                "baud_rate": baud_rate,
                "frame_id": frame_id,
                "publish_twist": publish_twist,
                "joint_topic": joint_topic,
                "pose_topic": pose_topic,
                "goal_topic": goal_topic,
                "ee_goal_topic": ee_goal_topic,
                "ee_pose_topic": ee_pose_topic,
                "ee_marker_topic": ee_marker_topic,
                "calibration_file": calibration_file,
            }
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyACM0",
                description="Serial port for SO-101 hardware",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="1000000",
                description="Serial baud rate for SO-101 hardware",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Frame id for joint states",
            ),
            DeclareLaunchArgument(
                "publish_twist",
                default_value="true",
                description="Publish pose delta as Twist",
            ),
            DeclareLaunchArgument(
                "joint_topic",
                default_value="/joint_states",
                description="Topic for JointState output",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/robot/cmd_pose",
                description="Topic for Twist output",
            ),
            DeclareLaunchArgument(
                "goal_topic",
                default_value="/arm_goal_ticks",
                description="Topic for incoming arm goals",
            ),
            DeclareLaunchArgument(
                "ee_goal_topic",
                default_value="/ee_goal",
                description="Topic for EE pose goal input",
            ),
            DeclareLaunchArgument(
                "ee_pose_topic",
                default_value="/ee_pose",
                description="Topic for FK EE pose output",
            ),
            DeclareLaunchArgument(
                "ee_marker_topic",
                default_value="/ee_marker",
                description="Topic for EE marker output",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lab8"), "config", "motor_calibration.yaml"]
                ),
                description="Calibration YAML with raw min/max per joint",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lab8"), "config", "lab8.rviz"]
                ),
                description="RViz config file",
            ),
            joint_state_reader,
            robot_state_publisher,
            rviz2,
        ]
    )
