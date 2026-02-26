from setuptools import setup

package_name = "mobile_robot_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/gazebo_launch.py",
            "launch/rviz_launch.py",
            "launch/bringup.launch.py",
        ]),
        ("share/" + package_name + "/rviz", [
            "rviz/trajectory.rviz",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Instructor",
    maintainer_email="instructor@university.edu",
    description="ROS2 mobile robot motion lab (Gazebo + cmd_vel + RViz path).",
    entry_points={
        "console_scripts": [
            "velocity_publisher = mobile_robot_lab.velocity_publisher:main",
            "square_path = mobile_robot_lab.square_path:main",
            "triangle_path = mobile_robot_lab.triangle_path:main",
            "odom_path_publisher = mobile_robot_lab.odom_path_publisher:main",
        ],
    },
)
