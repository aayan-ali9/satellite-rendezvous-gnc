
import numpy as np

from rendezvous.frames import lvlh_to_eci
from rendezvous.twobody import MU_EARTH, two_body_derivative
from rendezvous.nonlinear_control import (
    nonlinear_controlled_derivative
)


def test_nonlinear_control_direction_and_saturation():

    # Target positioned on positive ECI Y-axis
    radius = 6_878_000.0
    speed = np.sqrt(MU_EARTH / radius)
    n = np.sqrt(MU_EARTH / radius**3)

    target = np.array([
        0.0, radius, 0.0,
        -speed, 0.0, 0.0
    ])

    # Chaser is 100 m radially outward
    rho = np.array([100.0, 0.0, 0.0])
    rho_dot = np.zeros(3)

    r_chaser, v_chaser = lvlh_to_eci(
        target[:3],
        target[3:],
        rho,
        rho_dot,
        n
    )

    chaser = np.concatenate((r_chaser, v_chaser))
    combined_state = np.concatenate((target, chaser))

    # Create a simple test feedback gain
    K = np.zeros((3, 6))
    K[0, 0] = 2e-5

    waypoint = np.zeros(6)
    max_acceleration = 0.001

    # Requested radial acceleration is -0.002 m/s^2
    # Actual acceleration should saturate at -0.001
    derivative = nonlinear_controlled_derivative(
        0.0,
        combined_state,
        waypoint,
        n,
        K,
        max_acceleration
    )

    # Calculate gravitational acceleration without thrust
    gravity_only = two_body_derivative(0.0, chaser)

    # Isolate the acceleration supplied by the controller
    actual_thrust_eci = (
        derivative[9:12] - gravity_only[3:]
    )

    # Inward radial thrust points toward negative ECI Y
    expected_thrust_eci = np.array([
        0.0, -0.001, 0.0
    ])

    np.testing.assert_allclose(
        actual_thrust_eci,
        expected_thrust_eci,
        atol=1e-10,
        rtol=0
    )

    assert np.linalg.norm(actual_thrust_eci) <= (
        max_acceleration + 1e-10
    )
