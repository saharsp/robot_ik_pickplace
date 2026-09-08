"""
ik_node.py

ROS2 node that wraps the from-scratch SixAxisArm kinematics from
kinematics.py:

  - Subscribes to geometry_msgs/PoseStamped on "target_pose".
  - Solves IK for the requested pose (seeded from the current joint state,
    so motion is continuous rather than jumping to a fresh solution).
  - Publishes sensor_msgs/JointState on "joint_states" so robot_state_publisher
    can broadcast TF and RViz2 can visualize the arm moving.

This is intentionally a small, readable node -- the IK math itself lives
in kinematics.py so it stays testable without ROS2 (see standalone_demo/).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from robot_ik_pickplace.kinematics import SixAxisArm

JOINT_NAMES = [f"joint_{i+1}" for i in range(6)]


def pose_msg_to_matrix(pose_msg) -> np.ndarray:
    """Convert a geometry_msgs/Pose into a 4x4 homogeneous transform."""
    p = pose_msg.position
    q = pose_msg.orientation
    x, y, z, w = q.x, q.y, q.z, q.w

    # Quaternion -> rotation matrix
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        self.arm = SixAxisArm()
        self.current_q = np.zeros(self.arm.n_joints)

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.target_sub = self.create_subscription(
            PoseStamped, "target_pose", self._on_target_pose, 10
        )

        # Publish the current joint state at a steady rate so RViz2/TF stay live.
        self.timer = self.create_timer(0.05, self._publish_joint_state)

        self.get_logger().info(
            "ik_node ready. Publish a geometry_msgs/PoseStamped on "
            "'target_pose' to move the arm."
        )

    def _on_target_pose(self, msg: PoseStamped):
        target = pose_msg_to_matrix(msg.pose)
        q_solution, converged, iters = self.arm.inverse_kinematics(
            target, initial_guess=self.current_q
        )

        if not converged:
            self.get_logger().warn(
                f"IK did not converge in {iters} iterations for requested pose "
                f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
                f"{msg.pose.position.z:.3f}); using best-effort solution."
            )
        else:
            self.get_logger().info(
                f"IK converged in {iters} iterations for target "
                f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
                f"{msg.pose.position.z:.3f})."
            )

        self.current_q = q_solution

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.current_q.tolist()
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
