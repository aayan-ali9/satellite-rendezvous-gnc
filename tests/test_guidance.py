
import numpy as np

from rendezvous.guidance import waypoint_reached


def test_waypoint_reached():
    waypoint = np.array([0, -200, 0, 0, 0, 0])

    # Exactly at both acceptance limits
    state = np.array([3, -196, 0, 0.10, 0, 0])

    assert waypoint_reached(state, waypoint)


def test_waypoint_velocity_too_high():
    waypoint = np.array([0, -200, 0, 0, 0, 0])

    # Correct position, excessive relative velocity
    state = np.array([0, -200, 0, -0.15, 0.05, 0])

    assert not waypoint_reached(state, waypoint)


def test_waypoint_position_too_far():
    waypoint = np.array([0, -200, 0, 0, 0, 0])

    # Zero relative velocity, excessive position error
    state = np.array([6, -200, 0, 0, 0, 0])

    assert not waypoint_reached(state, waypoint)
