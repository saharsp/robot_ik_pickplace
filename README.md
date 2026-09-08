# robot_ik_pickplace

A from-scratch inverse kinematics solver and simulated pick-and-place
controller for a 6-DOF arm, built in Python and wired into ROS2.

**Why this project exists:** I spent 5 years programming ABB and KUKA
robots (RAPID/KRL) for pick-and-place and welding cells, using each
vendor's built-in kinematics solver and motion instructions. This project
reimplements that same kind of task — move to a pose, grasp, move to
another pose — but with the forward/inverse kinematics, the Jacobian, and
the motion sequencing all written from scratch in Python, wrapped in a
ROS2 node instead of a vendor teach-pendant program. It's a bridge project
from vendor robot programming into general-purpose robotics software.

## What's actually in here

- **`robot_ik_pickplace/kinematics.py`** — the core: a DH-parameter-based
  forward kinematics implementation, a geometric Jacobian, and a damped
  least-squares (Levenberg-Marquardt style) numerical inverse kinematics
  solver. Pure Python/NumPy, no ROS2 dependency, so it's testable and
  debuggable on its own.
- **`standalone_demo/demo_no_ros.py`** — runs the kinematics with no ROS2
  install required at all. Solves IK for a pick pose and a place pose and
  saves a 3D visualization (`ik_demo_result.png`). **Start here.**
- **`test/test_kinematics.py`** — pytest unit tests for the kinematics
  module (FK determinism, valid transform structure, IK convergence, an
  FK→IK round-trip check, Jacobian shape).
- **`robot_ik_pickplace/ik_node.py`** — a ROS2 node that subscribes to
  `geometry_msgs/PoseStamped` on `target_pose`, solves IK, and publishes
  `sensor_msgs/JointState` on `joint_states`.
- **`robot_ik_pickplace/pick_place_demo.py`** — a ROS2 node that publishes
  a scripted sequence of pick-and-place waypoints to `target_pose`, so you
  can watch the arm move through a full cycle.
- **`urdf/six_axis_arm.urdf`** — a simple 6-DOF arm model whose joint
  geometry is derived directly from the same DH parameters used in
  `kinematics.py` (verified to match exactly — see "Verifying the URDF
  matches the math" below). Swap this for a real robot's URDF (e.g. a UR5)
  once the pipeline works end to end.
- **`launch/ik_demo.launch.py`** — starts `robot_state_publisher`,
  `ik_node`, and RViz2 together.

## Quick start (no ROS2 needed)

This proves the math works before you touch any ROS2 setup:

```bash
cd standalone_demo
python3 demo_no_ros.py
```

This prints the IK solutions for a pick and a place target and saves
`ik_demo_result.png` showing the arm in its home, pick, and place
configurations.

Run the unit tests the same way:

```bash
python3 -m pytest test/test_kinematics.py -v
```

## Full ROS2 demo

Requires a ROS2 install (Humble/Jazzy or current LTS) with
`robot_state_publisher` and `rviz2`.

```bash
# from your colcon workspace's src/ directory
colcon build --packages-select robot_ik_pickplace
source install/setup.bash

# terminal 1
ros2 launch robot_ik_pickplace ik_demo.launch.py

# terminal 2
ros2 run robot_ik_pickplace pick_place_demo
```

In RViz2, add a `RobotModel` display (topic: `robot_description`) and a
`TF` display to watch the arm move through the pick-and-place sequence.

## Verifying the URDF matches the math

The URDF's joint origins were derived by hand from the DH table using the
standard DH→URDF conversion (each joint axis is `z`; each joint's fixed
origin encodes the *previous* row's `d, a, alpha`). This was checked
numerically — composing the URDF's joint transforms in code reproduces
`kinematics.py`'s forward kinematics output to floating-point precision.
If you change `DEFAULT_DH_TABLE`, update the URDF origins to match (or,
as a good follow-up project, write a small script that generates the URDF
directly from the DH table so the two can never drift apart).

## What to build next (roughly in order)

1. **Swap in a real robot's URDF** (e.g. `ur_description` for a UR5) and
   update `DEFAULT_DH_TABLE` to match its actual DH parameters, so the
   project reflects a real, named robot rather than an illustrative one.
2. **Gazebo integration** — actually simulate physics (gravity, contact,
   a simple gripper) instead of just kinematic visualization in RViz2.
3. **MoveIt2 comparison** — solve the same pick-and-place problem using
   MoveIt2's IK/planning stack, and be able to explain where a
   hand-rolled solver like this one would fall short of a
   production-grade planner (collision checking, joint-limit-aware
   planning, singularity handling, multiple IK solutions).
4. **Port the IK solver to C++** (with `rclcpp`) — most real control
   loops on hardware are C++, not Python; this closes that gap.
5. **Analytical IK** for this specific geometry, as a faster/more robust
   alternative to the numerical solver, if you want to go deeper on the
   math.

## Notes for anyone reading this as a portfolio piece

The illustrative DH parameters here (link lengths, wrist offset) are not
those of a specific commercial robot — this was built to demonstrate the
kinematics/IK/ROS2 pipeline itself, cleanly, rather than to exactly
replicate one vendor's arm. Step 1 above (swapping in a real robot's
published DH/URDF) is the natural next iteration.
