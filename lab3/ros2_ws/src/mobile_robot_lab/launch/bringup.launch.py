import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mobile_robot_lab")
    rviz_config = os.path.join(pkg_share, "rviz", "trajectory.rviz")

    return LaunchDescription([
        SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger"),

        # Gazebo simulation
        ExecuteProcess(
            cmd=["ros2", "launch", "turtlebot3_gazebo", "empty_world.launch.py"],
            output="screen",
        ),

        # Odometry -> Path for RViz
        Node(
            package="mobile_robot_lab",
            executable="odom_path_publisher",
            name="odom_path_publisher",
            output="screen",
        ),

        # RViz with config
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
