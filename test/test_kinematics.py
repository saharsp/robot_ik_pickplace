"""
test_kinematics.py

Pure-Python unit tests for the from-scratch kinematics module -- no ROS2
required to run these. This is the kind of test suite worth having before
you ever touch hardware or even a simulator: prove the math is correct in
isolation first.

Run:
    cd robot_ik_pickplace/robot_ik_pickplace   (or adjust the path below)
    python3 -m pytest ../test/test_kinematics.py -v

(or, once this is a real colcon package: `colcon test --packages-select robot_ik_pickplace`)
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "robot_ik_pickplace"))

import numpy as np
from kinematics import SixAxisArm, pose_from_xyz


def test_forward_kinematics_home_position_is_deterministic():
    arm = SixAxisArm()
    q = np.zeros(6)
    pose_a = arm.end_effector_pose(q)
    pose_b = arm.end_effector_pose(q)
    assert np.allclose(pose_a, pose_b)


def test_forward_kinematics_output_is_valid_homogeneous_transform():
    arm = SixAxisArm()
    q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    T = arm.end_effector_pose(q)
    assert T.shape == (4, 4)
    # Bottom row must be [0, 0, 0, 1]
    assert np.allclose(T[3, :], [0, 0, 0, 1])
    # Rotation part must be orthonormal (R^T R = I)
    R = T[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-8)


def test_inverse_kinematics_converges_for_reachable_target():
    arm = SixAxisArm()
    target = pose_from_xyz(0.5, 0.2, 0.4)
    q_solution, converged, iters = arm.inverse_kinematics(target)
    assert converged, f"IK failed to converge in {iters} iterations"

    achieved = arm.end_effector_pose(q_solution)
    assert np.allclose(achieved[:3, 3], target[:3, 3], atol=1e-3)


def test_inverse_kinematics_roundtrip_matches_forward_kinematics():
    """
    Pick a random joint configuration, compute its pose via FK, then solve
    IK for that same pose and confirm the resulting pose matches (note: the
    joint angles themselves need not match, since redundant/alternate
    solutions can reach the same pose -- what must match is the pose).
    """
    arm = SixAxisArm()
    rng = np.random.default_rng(seed=42)
    q_true = rng.uniform(-1.0, 1.0, size=6)

    target_pose = arm.end_effector_pose(q_true)
    q_solution, converged, _ = arm.inverse_kinematics(target_pose, initial_guess=q_true * 0.9)

    assert converged
    achieved_pose = arm.end_effector_pose(q_solution)
    assert np.allclose(achieved_pose, target_pose, atol=1e-3)


def test_jacobian_shape():
    arm = SixAxisArm()
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    J = arm.geometric_jacobian(q)
    assert J.shape == (6, 6)


if __name__ == "__main__":
    # Allow running as a plain script too (without pytest installed).
    test_forward_kinematics_home_position_is_deterministic()
    test_forward_kinematics_output_is_valid_homogeneous_transform()
    test_inverse_kinematics_converges_for_reachable_target()
    test_inverse_kinematics_roundtrip_matches_forward_kinematics()
    test_jacobian_shape()
    print("All tests passed.")
