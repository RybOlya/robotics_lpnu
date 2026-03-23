from setuptools import setup, find_packages
import os
from glob import glob

package_name = "lab6"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy]*"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*"))),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*.sdf"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Instructor",
    maintainer_email="instructor@lpnu.ua",
    description="Lab 6: Nav2 motion planning",
    license="Apache-2.0",
    tests_require=["pytest"],
)
