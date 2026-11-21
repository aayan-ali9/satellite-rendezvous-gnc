
"""Tests for the first Kalman-filter extension (not yet closed-loop)."""

import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.constants import circular_mean_motion
from rendezvous.hcw import analytical_uncontrolled, hcw_derivative
from rendezvous.kalman import HCWKalmanFilter, discrete_hcw_model


N = circular_mean_motion(500_000)
DT = 10.0


def test_discrete_hcw_matches_analytical_motion():
    initial = np.array([
        100.0, -995.0, 20.0,
        0.03, -0.01, 0.02
    ])

    F, G = discrete_hcw_model(N, DT)
    predicted = F @ initial

    expected = analytical_uncontrolled(
        initial, np.array([DT]), N
    )[0]

    np.testing.assert_allclose(
        predicted, expected,
        atol=1e-9, rtol=1e-10
    )
    assert G.shape == (6, 3)


def test_discrete_control_matches_numerical_hcw():
    initial = np.array([
        25.0, -100.0, 5.0,
        0.01, 0.0, -0.01
    ])

    control = np.array([1e-4, -2e-4, 3e-4])
    F, G = discrete_hcw_model(N, DT)

    predicted = F @ initial + G @ control

    solution = solve_ivp(
        lambda t, state: hcw_derivative(
            t, state, N, control
        ),
        (0.0, DT),
        initial,
        rtol=1e-11,
        atol=1e-12,
    )

    assert solution.success

    np.testing.assert_allclose(
        predicted, solution.y[:, -1],
        atol=1e-8
    )


def test_filter_reduces_noisy_position_error_and_recovers_velocity():
    rng = np.random.default_rng(42)

    initial_truth = np.array([
        100.0, -995.0, 10.0,
        0.02, -0.01, 0.005
    ])

    filter_ = HCWKalmanFilter(
        mean_motion=N,
        dt=DT,
        initial_state=initial_truth + np.array([
            10.0, -8.0, 6.0,
            0.15, -0.10, 0.10
        ]),
        initial_covariance=np.diag(
            [20.0**2] * 3 + [0.3**2] * 3
        ),
        position_noise_std=5.0,
        acceleration_noise_std=1e-5,
    )

    truth = initial_truth.copy()
    raw_errors = []
    estimated_errors = []
    velocity_errors = []

    for i in range(200):
        truth = filter_.F @ truth

        measured = (
            truth[:3] +
            rng.normal(0.0, 5.0, size=3)
        )

        filter_.predict(np.zeros(3))
        estimated = filter_.update(measured)

        if i >= 50:
            # Evaluate after the estimator has settled.
            raw_errors.append(
                np.linalg.norm(measured - truth[:3])
            )
            estimated_errors.append(
                np.linalg.norm(estimated[:3] - truth[:3])
            )
            velocity_errors.append(
                np.linalg.norm(estimated[3:] - truth[3:])
            )

    assert np.mean(estimated_errors) < np.mean(raw_errors)
    assert np.mean(velocity_errors) < 0.02

    np.testing.assert_allclose(
        filter_.covariance,
        filter_.covariance.T,
        atol=1e-10
    )

    assert np.linalg.eigvalsh(
        filter_.covariance
    ).min() > -1e-10
