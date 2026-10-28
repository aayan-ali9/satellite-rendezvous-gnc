
import numpy as np


def lvlh_basis(r_target, v_target):
    """
    Return the LVLH-to-ECI rotation matrix.

    LVLH axes:
        x: radial outward
        y: along-track
        z: orbit normal

    Inputs are target ECI position and velocity.
    """

    r_target = np.asarray(r_target, dtype=float)
    v_target = np.asarray(v_target, dtype=float)

    # Radial unit vector
    e_x = r_target / np.linalg.norm(r_target)

    # Orbit-normal unit vector
    angular_momentum = np.cross(r_target, v_target)
    e_z = angular_momentum / np.linalg.norm(angular_momentum)

    # Along-track unit vector
    e_y = np.cross(e_z, e_x)

    # Columns are LVLH axes expressed in ECI
    C = np.column_stack((e_x, e_y, e_z))

    return C

def lvlh_to_eci(r_target, v_target, rho, rho_dot, n):
    """
    Convert relative LVLH position and velocity into
    absolute ECI position and velocity.

    Assumes a circular target orbit with angular rate n.
    """

    C = lvlh_basis(r_target, v_target)

    omega = np.array([0.0, 0.0, n])

    r_chaser = r_target + C @ rho

    v_chaser = v_target + C @ (
        rho_dot + np.cross(omega, rho)
    )

    return r_chaser, v_chaser

def eci_to_lvlh(r_target, v_target,
                r_chaser, v_chaser, n):
    """
    Convert absolute ECI spacecraft states into
    the chaser's relative LVLH position and velocity.

    Assumes a circular target orbit with angular rate n.
    """

    C = lvlh_basis(r_target, v_target)

    omega = np.array([0.0, 0.0, n])

    # Relative ECI position and velocity
    relative_r = r_chaser - r_target
    relative_v = v_chaser - v_target

    # Convert position to LVLH
    rho = C.T @ relative_r

    # Remove the rotating-frame velocity contribution
    rho_dot = (
        C.T @ relative_v
        - np.cross(omega, rho)
    )

    return rho, rho_dot
