
import numpy as np

from rendezvous.frames import (
    lvlh_basis,
    lvlh_to_eci,
    eci_to_lvlh
)
from rendezvous.twobody import (
    MU_EARTH,
    two_body_derivative
)


def test_lvlh_rotation_matrix():

    r = np.array([0.0, 6_878_000.0, 0.0])
    v = np.array([-7600.0, 0.0, 0.0])

    C = lvlh_basis(r, v)

    # Rotation matrices must preserve vector lengths
    np.testing.assert_allclose(
        C.T @ C,
        np.eye(3),
        atol=1e-12
    )

    # Proper rotations must have determinant +1
    assert np.isclose(np.linalg.det(C), 1.0)


def test_lvlh_eci_round_trip():

    radius = 6_878_000.0
    n = np.sqrt(MU_EARTH / radius**3)

    r_target = np.array([0.0, radius, 0.0])
    v_target = np.array([
        -np.sqrt(MU_EARTH / radius), 0.0, 0.0
    ])

    rho = np.array([100.0, -50.0, 25.0])
    rho_dot = np.array([0.02, -0.03, 0.01])

    r_chaser, v_chaser = lvlh_to_eci(
        r_target, v_target, rho, rho_dot, n
    )

    rho_recovered, velocity_recovered = eci_to_lvlh(
        r_target, v_target,
        r_chaser, v_chaser, n
    )

    np.testing.assert_allclose(
        rho_recovered,
        rho,
        atol=1e-8,
        rtol=0
    )

    np.testing.assert_allclose(
        velocity_recovered,
        rho_dot,
        atol=1e-11,
        rtol=0
    )


def test_two_body_gravity():

    radius = 6_878_000.0

    state = np.array([
        radius, 0.0, 0.0,
        0.0, 7600.0, 0.0
    ])

    derivative = two_body_derivative(0.0, state)

    expected_acceleration = np.array([
        -MU_EARTH / radius**2,
        0.0,
        0.0
    ])

    np.testing.assert_allclose(
        derivative[:3],
        state[3:],
        atol=1e-12
    )

    np.testing.assert_allclose(
        derivative[3:],
        expected_acceleration,
        atol=1e-12
    )
