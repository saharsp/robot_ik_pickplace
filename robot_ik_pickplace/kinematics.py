"""
kinematics.py

From-scratch forward and inverse kinematics for a 6-DOF revolute arm,
using standard Denavit-Hartenberg (DH) parameters.

This module has NO ROS2 dependency on purpose: it is pure Python/NumPy so
you can develop, test, and debug the math in isolation before wiring it
into a ROS2 node. This is a good practice in general -- keep your control
math testable outside the middleware.

DH convention used (standard, not "modified"):
    A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)

Author: <your name> -- built as a portfolio project translating ABB/KUKA
robot programming experience into from-scratch robotics software.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class DHParam:
    """One row of a Denavit-Hartenberg parameter table."""
    theta_offset: float  # fixed offset added to the joint variable (rad)
    d: float             # link offset along previous z (m)
    a: float              # link length along new x (m)
    alpha: float         # link twist about new x (rad)


# Example 6-DOF anthropomorphic arm with a spherical wrist.
# Lengths are illustrative -- swap these for a real robot's datasheet
# values (e.g. a UR5) once you're ready to match a specific URDF exactly.
DEFAULT_DH_TABLE = [
    DHParam(theta_offset=0.0, d=0.15, a=0.0,  alpha=np.pi / 2),
    DHParam(theta_offset=0.0, d=0.0,  a=0.50, alpha=0.0),
    DHParam(theta_offset=0.0, d=0.0,  a=0.40, alpha=0.0),
    DHParam(theta_offset=0.0, d=0.10, a=0.0,  alpha=np.pi / 2),
    DHParam(theta_offset=0.0, d=0.10, a=0.0,  alpha=-np.pi / 2),
    DHParam(theta_offset=0.0, d=0.10, a=0.0,  alpha=0.0),
]

JOINT_LIMITS = [(-np.pi, np.pi)] * 6  # radians; tighten to match a real robot


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Homogeneous transform for one DH row."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


class SixAxisArm:
    """Forward/inverse kinematics for a 6-revolute-joint serial arm."""

    def __init__(self, dh_table=None, joint_limits=None):
        self.dh_table = dh_table if dh_table is not None else DEFAULT_DH_TABLE
        self.joint_limits = joint_limits if joint_limits is not None else JOINT_LIMITS
        self.n_joints = len(self.dh_table)

    # ---------- Forward kinematics ----------

    def forward_kinematics(self, joint_angles) -> list:
        """
        Returns the list of cumulative homogeneous transforms T_0_1 .. T_0_n,
        i.e. transforms[-1] is the end-effector pose in the base frame.
        Also useful for plotting each joint's position along the chain.
        """
        joint_angles = np.asarray(joint_angles, dtype=float)
        assert len(joint_angles) == self.n_joints

        transforms = []
        T = np.eye(4)
        for q, dh in zip(joint_angles, self.dh_table):
            A = dh_transform(q + dh.theta_offset, dh.d, dh.a, dh.alpha)
            T = T @ A
            transforms.append(T.copy())
        return transforms

    def end_effector_pose(self, joint_angles) -> np.ndarray:
        return self.forward_kinematics(joint_angles)[-1]

    def joint_positions(self, joint_angles) -> np.ndarray:
        """Cartesian xyz of each joint origin, including the base at index 0."""
        transforms = self.forward_kinematics(joint_angles)
        positions = [np.zeros(3)] + [T[:3, 3] for T in transforms]
        return np.array(positions)

    # ---------- Jacobian ----------

    def geometric_jacobian(self, joint_angles) -> np.ndarray:
        """
        6xN geometric Jacobian (linear velocity rows 0-2, angular rows 3-5)
        for revolute joints, computed directly from the FK chain:
            Jv_i = z_(i-1) x (o_n - o_(i-1))
            Jw_i = z_(i-1)
        """
        transforms = self.forward_kinematics(joint_angles)
        o_n = transforms[-1][:3, 3]

        z_axes = [np.array([0.0, 0.0, 1.0])]  # z0 in base frame
        origins = [np.zeros(3)]               # o0 in base frame
        for T in transforms[:-1]:
            z_axes.append(T[:3, 2])
            origins.append(T[:3, 3])

        J = np.zeros((6, self.n_joints))
        for i in range(self.n_joints):
            z_im1 = z_axes[i]
            o_im1 = origins[i]
            J[0:3, i] = np.cross(z_im1, o_n - o_im1)
            J[3:6, i] = z_im1
        return J

    # ---------- Inverse kinematics ----------

    def inverse_kinematics(
        self,
        target_pose: np.ndarray,
        initial_guess=None,
        max_iters: int = 200,
        tol: float = 1e-4,
        damping: float = 0.05,
    ):
        """
        Damped least-squares (Levenberg-Marquardt style) numerical IK.

        This avoids the singularity blow-ups of a plain pseudo-inverse
        Jacobian solver, at the cost of needing a damping term. It is a
        standard, well-documented approach and a reasonable starting point
        before you optionally add an analytical solver for a specific
        robot geometry.

        Returns (joint_angles, converged: bool, iterations: int).
        """
        q = (
            np.array(initial_guess, dtype=float)
            if initial_guess is not None
            else np.zeros(self.n_joints)
        )

        for it in range(max_iters):
            current_pose = self.end_effector_pose(q)
            error = self._pose_error(current_pose, target_pose)

            if np.linalg.norm(error) < tol:
                return self._clip_to_limits(q), True, it

            J = self.geometric_jacobian(q)
            JJt = J @ J.T
            damped = JJt + (damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(damped, error)
            q = q + dq

        return self._clip_to_limits(q), False, max_iters

    def _clip_to_limits(self, q: np.ndarray) -> np.ndarray:
        return np.array([
            np.clip(qi, lo, hi) for qi, (lo, hi) in zip(q, self.joint_limits)
        ])

    @staticmethod
    def _pose_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
        """6-vector error: 3 position + 3 orientation (axis-angle approx)."""
        pos_err = target[:3, 3] - current[:3, 3]

        R_err = target[:3, :3] @ current[:3, :3].T
        # Small-angle axis-angle extraction from the skew-symmetric part.
        rot_err = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ])
        return np.concatenate([pos_err, rot_err])


def pose_from_xyz(x: float, y: float, z: float, R: np.ndarray = None) -> np.ndarray:
    """Convenience: build a 4x4 pose with identity (or given) orientation."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T
