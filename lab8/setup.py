import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lab8"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial", "PyYAML"],
    zip_safe=True,
    maintainer="Instructor",
    maintainer_email="instructor@lpnu.ua",
    description="Lab 8: SO-101 serial bridge and goal tools",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_reader = lab8.joint_state_reader:main",
            "send_goal = lab8.send_goal:main",
            "run_motion = lab8.run_motion:main",
            "send_ee_goal = lab8.send_ee_goal:main",
        ]
    },
)
