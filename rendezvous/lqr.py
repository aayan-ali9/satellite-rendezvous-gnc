
import numpy as np
from scipy.linalg import solve_continuous_are


def hcw_state_space(n):
    """Build HCW state and acceleration-input matrices."""
    A = np.zeros((6, 6))

    # Position derivatives = velocities
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0

    # HCW accelerations
    A[3, 0] = 3 * n**2
    A[3, 4] = 2 * n
    A[4, 3] = -2 * n
    A[5, 2] = -n**2

    # Three independent commanded accelerations
    B = np.zeros((6, 3))
    B[3:6, :] = np.eye(3)

    return A, B


def lqr_gain(A, B, Q, R):
    """Compute continuous-time LQR feedback gains."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K
