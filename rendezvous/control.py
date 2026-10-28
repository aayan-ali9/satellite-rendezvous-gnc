
import numpy as np


def saturate_acceleration(requested, max_acceleration):
    """
    Limit the total commanded acceleration magnitude.

    Parameters
    ----------
    requested : array, shape (3,)
        Requested LVLH acceleration [ux, uy, uz] in m/s^2.
    max_acceleration : float
        Maximum total acceleration magnitude in m/s^2.
    """

    requested = np.asarray(requested, dtype=float)

    if requested.shape != (3,):
        raise ValueError("Acceleration must have 3 components.")

    if not np.all(np.isfinite(requested)):
        raise ValueError("Acceleration must contain finite values.")

    if not np.isfinite(max_acceleration) or max_acceleration <= 0:
        raise ValueError("Maximum acceleration must be positive and finite.")

    magnitude = np.linalg.norm(requested)

    # No change if the request is within the limit
    if magnitude <= max_acceleration:
        return requested.copy()

    # Scale the vector while preserving its direction
    return requested * (max_acceleration / magnitude)
