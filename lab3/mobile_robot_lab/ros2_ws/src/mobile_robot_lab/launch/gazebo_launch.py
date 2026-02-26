from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # TurtleBot3 Gazebo needs model selection
    return LaunchDescription([
        SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger"),
        ExecuteProcess(
            cmd=["ros2", "launch", "turtlebot3_gazebo", "empty_world.launch.py"],
            output="screen",
        ),
    ])
