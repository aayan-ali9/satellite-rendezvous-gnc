
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.constants import circular_mean_motion
from rendezvous.lqr import hcw_state_space, lqr_gain


def build_controller():
    n = circular_mean_motion(500_000)
    A, B = hcw_state_space(n)

    Q = np.diag([1e-6, 1e-6, 1e-6,
                 1e-2, 1e-2, 1e-2])
    R = np.eye(3) * 1e6

    K = lqr_gain(A, B, Q, R)
    return n, A, B, K


def test_state_space_dimensions():
    _, A, B, K = build_controller()

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    assert K.shape == (3, 6)


def test_closed_loop_stability():
    _, A, B, K = build_controller()

    eigenvalues = np.linalg.eigvals(A - B @ K)

    assert np.all(np.real(eigenvalues) < 0)


def test_lqr_convergence():
    n, A, B, K = build_controller()

    initial_state = np.array(
        [100, -995, 0, 0, 0, 0], dtype=float
    )
    desired_state = np.array(
        [0, -20, 0, 0, 0, 0], dtype=float
    )

    initial_error = initial_state - desired_state

    duration = 4 * (2 * np.pi / n)

    # Propagate the closed-loop error dynamics
    def derivative(t, error):
        return (A - B @ K) @ error

    solution = solve_ivp(
        derivative,
        (0, duration),
        initial_error,
        rtol=1e-9,
        atol=1e-11
    )

    assert solution.success

    final_error = solution.y[:, -1]

    assert np.linalg.norm(final_error[:3]) < 0.01
    assert np.linalg.norm(final_error[3:]) < 1e-4
