
import numpy as np

from rendezvous.guidance import waypoint_acceptance_margin


waypoint = np.array([0, -200, 0, 0, 0, 0], dtype=float)


def margin(state):
    return waypoint_acceptance_margin(
        np.array(state, dtype=float),
        waypoint,
        5.0,
        0.10
    )


def test_waypoint_accepts_valid_state():
    state = [2, -200, 0, 0, 0.05, 0]

    assert margin(state) < 0


def test_waypoint_rejects_high_velocity():
    state = [0, -200, 0, 0, 0.11, 0]

    assert margin(state) > 0


def test_waypoint_accepts_exact_boundary():
    state = [5, -200, 0, 0, 0, 0]

    assert np.isclose(margin(state), 0.0)
