"""TurtleBot3 in lab6 room_nav2.sdf + Nav2 (localization + navigation)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lab6_share = get_package_share_directory("lab6")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = os.path.join(lab6_share, "worlds", "room_nav2.sdf")
    map_yaml = os.path.join(lab6_share, "maps", "room_nav2.yaml")
    params_file = os.path.join(lab6_share, "config", "nav2_params.yaml")

    default_rviz = os.path.join(
        nav2_bringup_share, "rviz", "nav2_default_view.rviz"
    )
    rviz_cfg = LaunchConfiguration("rviz_config").perform(context)
    if not rviz_cfg:
        rviz_cfg = default_rviz

    use_rviz = LaunchConfiguration("rviz").perform(context) == "true"

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v2 "}.items(),
    )

    rsp_launch = os.path.join(
        turtlebot3_gazebo_share, "launch", "robot_state_publisher.launch.py"
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    spawn_launch = os.path.join(
        turtlebot3_gazebo_share, "launch", "spawn_turtlebot3.launch.py"
    )
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch),
        launch_arguments={"x_pose": "0.0", "y_pose": "0.0"}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": map_yaml,
            "use_sim_time": "true",
            "params_file": params_file,
            "autostart": "true",
            # Separate processes: correct per-node params (e.g. enable_stamped_cmd_vel) and clearer logs.
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    actions = [
        gzserver,
        gzclient,
        spawn_turtlebot,
        robot_state_publisher,
        nav2,
    ]
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    default_rviz = os.path.join(
        nav2_bringup_share, "rviz", "nav2_default_view.rviz"
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger"),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(
                    get_package_share_directory("turtlebot3_gazebo"),
                    "models",
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 with Nav2 view",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
