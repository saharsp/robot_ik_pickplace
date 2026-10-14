import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "robot_ik_pickplace"))

import math
from path_metrics import path_length


def test_path_length_empty_list_is_zero():
    assert path_length([]) == 0.0


def test_path_length_single_waypoint_is_zero():
    assert path_length([("home", 0.9, -0.2, 0.05)]) == 0.0


def test_path_length_two_waypoints_matches_euclidean_distance():
    waypoints = [("a", 0.0, 0.0, 0.0), ("b", 3.0, 4.0, 0.0)]
    assert math.isclose(path_length(waypoints), 5.0)  # 3-4-5 triangle


def test_path_length_sums_consecutive_segments_not_endpoint_to_endpoint():
    waypoints = [("a", 0.0, 0.0, 0.0), ("b", 1.0, 0.0, 0.0), ("c", 1.0, 1.0, 0.0)]
    assert math.isclose(path_length(waypoints), 2.0)
    assert not math.isclose(path_length(waypoints), math.sqrt(2))
