"""
ik_demo.launch.py

Starts:
  - robot_state_publisher (reads the URDF, broadcasts TF from joint_states)
  - ik_node (from-scratch IK solver, publishes joint_states)
  - rviz2 (visualize the arm)

Run with:
    ros2 launch robot_ik_pickplace ik_demo.launch.py

Then, in a second terminal:
    ros2 run robot_ik_pickplace pick_place_demo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_ik_pickplace")
    urdf_path = os.path.join(pkg_share, "urdf", "six_axis_arm.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="robot_ik_pickplace",
            executable="ik_node",
            name="ik_node",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
