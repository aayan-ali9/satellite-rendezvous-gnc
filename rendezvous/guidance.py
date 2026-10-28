
import numpy as np


def waypoint_reached(
    state,
    waypoint,
    position_tolerance=5.0,
    velocity_tolerance=0.10,
):
    """Check whether a spacecraft has reached a waypoint."""

    # Position error magnitude (m)
    position_error = np.linalg.norm(
        state[:3] - waypoint[:3]
    )

    # Relative velocity error magnitude (m/s)
    velocity_error = np.linalg.norm(
        state[3:] - waypoint[3:]
    )

    return (
        position_error <= position_tolerance
        and velocity_error <= velocity_tolerance
    )

def waypoint_acceptance_margin(
    relative_state,
    waypoint,
    position_tolerance,
    velocity_tolerance
):
    """
    Negative or zero: waypoint accepted.
    Positive: waypoint not yet accepted.
    """

    position_error = np.linalg.norm(
        relative_state[:3] - waypoint[:3]
    )

    velocity_error = np.linalg.norm(
        relative_state[3:] - waypoint[3:]
    )

    return max(
        position_error / position_tolerance,
        velocity_error / velocity_tolerance
    ) - 1.0
