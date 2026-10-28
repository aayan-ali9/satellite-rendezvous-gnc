
import numpy as np

MU_EARTH = 3.986004418e14  # m^3/s^2


def two_body_derivative(t, state):
    """
    Calculate a spacecraft's ECI state derivative
    under Earth's two-body gravitational field.

    State:
        [x, y, z, vx, vy, vz]

    Units:
        Position: m
        Velocity: m/s
        Acceleration: m/s^2
    """

    r = state[:3]
    v = state[3:]

    radius = np.linalg.norm(r)

    if radius == 0:
        raise ValueError("Spacecraft position cannot be zero.")

    acceleration = -MU_EARTH * r / radius**3

    return np.concatenate((v, acceleration))
