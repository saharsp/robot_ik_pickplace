"""
demo_no_ros.py

Runs and visualizes the from-scratch kinematics WITHOUT ROS2 installed.
Use this to develop/debug the math first -- get this working before you
touch the ROS2 wiring at all. This is exactly the order a real robotics
software project should go in: prove the algorithm in isolation, then
integrate it.

Run:
    python3 demo_no_ros.py

Requires: numpy, matplotlib
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "robot_ik_pickplace"))

import numpy as np
import matplotlib.pyplot as plt
from kinematics import SixAxisArm, pose_from_xyz


def plot_arm(ax, joint_positions, color, label):
    xs, ys, zs = joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2]
    ax.plot(xs, ys, zs, "-o", color=color, label=label, linewidth=2, markersize=5)


def main():
    arm = SixAxisArm()

    # 1. Forward kinematics sanity check: a simple pose, print the resulting
    #    end-effector transform.
    home_q = np.zeros(6)
    home_pose = arm.end_effector_pose(home_q)
    print("Home joint angles:", home_q)
    print("End-effector pose at home:\n", np.round(home_pose, 3))

    # 2. Inverse kinematics: ask the arm to reach a target point (a "pick"
    #    location), starting from the home configuration.
    pick_target = pose_from_xyz(0.5, 0.3, 0.4)
    place_target = pose_from_xyz(-0.4, 0.4, 0.3)

    print("\nSolving IK for pick target...")
    q_pick, converged, iters = arm.inverse_kinematics(pick_target, initial_guess=home_q)
    print(f"  converged={converged} in {iters} iterations")
    print("  joint angles (rad):", np.round(q_pick, 3))
    achieved = arm.end_effector_pose(q_pick)
    print("  achieved position:", np.round(achieved[:3, 3], 4))
    print("  target position:  ", np.round(pick_target[:3, 3], 4))

    print("\nSolving IK for place target (seeded from pick solution)...")
    q_place, converged2, iters2 = arm.inverse_kinematics(place_target, initial_guess=q_pick)
    print(f"  converged={converged2} in {iters2} iterations")
    print("  joint angles (rad):", np.round(q_place, 3))

    # 3. Visualize: home pose, pick pose, place pose, all on one 3D plot.
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    plot_arm(ax, arm.joint_positions(home_q), "tab:gray", "Home")
    plot_arm(ax, arm.joint_positions(q_pick), "tab:blue", "Pick pose (IK solution)")
    plot_arm(ax, arm.joint_positions(q_place), "tab:orange", "Place pose (IK solution)")

    ax.scatter(*pick_target[:3, 3], color="tab:blue", marker="*", s=200, label="Pick target")
    ax.scatter(*place_target[:3, 3], color="tab:orange", marker="*", s=200, label="Place target")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("From-scratch 6-DOF IK: pick-and-place configurations")
    ax.legend()
    ax.set_box_aspect([1, 1, 1])

    out_path = os.path.join(os.path.dirname(__file__), "ik_demo_result.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\nSaved visualization to {out_path}")


if __name__ == "__main__":
    main()
