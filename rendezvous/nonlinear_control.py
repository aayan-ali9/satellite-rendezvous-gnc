
import numpy as np

from rendezvous.twobody import two_body_derivative
from rendezvous.frames import lvlh_basis, eci_to_lvlh
from rendezvous.control import saturate_acceleration


def nonlinear_controlled_derivative(
    t, state, waypoint, n, K, max_acceleration
):
    """
    Propagate target and controlled chaser in ECI.

    Combined 12-element state:
        [target position (3),
         target velocity (3),
         chaser position (3),
         chaser velocity (3)]

    LQR feedback is calculated in LVLH.
    Gravitational propagation occurs in ECI.
    """

    # Separate the two spacecraft states
    target = state[:6]
    chaser = state[6:]

    r_target = target[:3]
    v_target = target[3:]

    r_chaser = chaser[:3]
    v_chaser = chaser[3:]

    # Convert chaser's ECI state into relative LVLH
    rho, rho_dot = eci_to_lvlh(
        r_target, v_target,
        r_chaser, v_chaser, n
    )

    relative_state = np.concatenate((rho, rho_dot))

    # Calculate LQR acceleration in LVLH
    error = relative_state - waypoint
    requested_control = -K @ error

    # Apply the spacecraft acceleration limit
    actual_control_lvlh = saturate_acceleration(
        requested_control,
        max_acceleration
    )

    # Convert commanded acceleration into ECI
    C = lvlh_basis(r_target, v_target)
    actual_control_eci = C @ actual_control_lvlh

    # Independently calculate gravitational motion
    target_derivative = two_body_derivative(t, target)
    chaser_derivative = two_body_derivative(t, chaser)

    # Only the chaser receives control acceleration
    chaser_derivative[3:] += actual_control_eci

    return np.concatenate((
        target_derivative,
        chaser_derivative
    ))
