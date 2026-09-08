"""
pick_place_demo.py

Sends a scripted pick-and-place sequence of target poses to ik_node by
publishing geometry_msgs/PoseStamped messages on "target_pose", with pauses
between waypoints so you can watch the arm move in RViz2.

Sequence: home -> approach pick -> pick -> lift -> approach place -> place
-> retreat -> home.

This mirrors the shape of a real pick-and-place cycle you'd have written
in RAPID or KRL, just expressed as a sequence of ROS2 messages instead of
vendor motion instructions.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def make_pose(x, y, z):
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0  # identity orientation for simplicity
    return msg


WAYPOINTS = [
    ("approach pick", 0.5, 0.3, 0.55),
    ("pick",          0.5, 0.3, 0.40),
    ("lift",          0.5, 0.3, 0.55),
    ("approach place", -0.4, 0.4, 0.45),
    ("place",          -0.4, 0.4, 0.30),
    ("retreat",        -0.4, 0.4, 0.45),
    ("home",            0.9, -0.2, 0.05),
]


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("pick_place_demo")
        self.pub = self.create_publisher(PoseStamped, "target_pose", 10)

    def run_sequence(self, dwell_seconds: float = 2.0):
        for label, x, y, z in WAYPOINTS:
            self.get_logger().info(f"-> {label}: ({x}, {y}, {z})")
            msg = make_pose(x, y, z)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(dwell_seconds)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceDemo()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
