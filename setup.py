import os
from glob import glob
from setuptools import find_packages, setup

package_name = "robot_ik_pickplace"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description=(
        "From-scratch IK + simulated pick-and-place demo for a 6-DOF arm, "
        "built to bridge ABB/KUKA robot programming into ROS2/Python."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_node = robot_ik_pickplace.ik_node:main",
            "pick_place_demo = robot_ik_pickplace.pick_place_demo:main",
        ],
    },
)
