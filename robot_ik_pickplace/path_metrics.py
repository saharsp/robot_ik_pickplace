"""
path_metrics.py

Small, pure-Python/NumPy utilities for reasoning about a planned sequence of
Cartesian waypoints -- no ROS2 dependency, same philosophy as kinematics.py.
"""

import numpy as np


def path_length(waypoints: list[tuple[float, float, float, float]]) -> float:
    
    """
    Given a list of (label, x, y, z) waypoints, like WAYPOINTS in
    pick_place_demo.py, return the total Euclidean distance traveled
    by the end-effector moving through them in order.
    """
    if len(waypoints) < 2:
        return 0.0
    
    # 1. Extract only the x, y, z coordinates (skip the text label at index 0)
    coords = np.array([wp[1:] for wp in waypoints])
    
    # 2. Compute differences between consecutive points along the path
    diffs = np.diff(coords, axis=0)
    
    # 3. Calculate the Euclidean distance for each step and sum them up
    distances = np.sqrt(np.sum(diffs**2, axis=1))
    
    return float(np.sum(distances))
    
    
WAYPOINTS = [
    ("approach pick", 0.5, 0.3, 0.55),
    ("pick",          0.5, 0.3, 0.40),
    ("lift",          0.5, 0.3, 0.55),
    ("approach place", -0.4, 0.4, 0.45),
    ("place",          -0.4, 0.4, 0.30),
    ("retreat",        -0.4, 0.4, 0.45),
    ("home",            0.9, -0.2, 0.05),
]


print(f"Path length: {path_length(WAYPOINTS):.4f}")
