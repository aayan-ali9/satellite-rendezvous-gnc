"""Verify the matrix milestone against the existing ODE and closed-form model."""

import numpy as np
import pytest
from scipy.linalg import expm

from rendezvous.constants import circular_mean_motion, orbital_period
from rendezvous.hcw import (
    analytical_uncontrolled, hcw_derivative, hcw_state_space,
    propagate_uncontrolled,
)

N = circular_mean_motion(500_000.0)


def test_matrix_derivative_matches_every_state_and_input_component() -> None:
    a, b = hcw_state_space(N)
    assert a.shape == (6, 6)
    assert b.shape == (6, 3)
    # Basis vectors cover all columns without relying on cancellation in one state.
    for state in np.eye(6):
        np.testing.assert_allclose(a @ state, hcw_derivative(0.0, state, N))
    for acceleration in np.eye(3):
        np.testing.assert_allclose(
            b @ acceleration, hcw_derivative(0.0, np.zeros(6), N, acceleration),
        )


def test_matrix_exponential_matches_closed_form() -> None:
    a, _ = hcw_state_space(N)
    times = np.array([0.0, 60.0, orbital_period(N) / 4, 2 * orbital_period(N)])
    for initial in np.eye(6):
        matrix_states = np.array([expm(a * time) @ initial for time in times])
        np.testing.assert_allclose(
            matrix_states, analytical_uncontrolled(initial, times, N),
            rtol=1e-10, atol=1e-8,
        )


def test_three_acceleration_axes_make_all_six_states_controllable() -> None:
    a, b = hcw_state_space(N)
    # These first six columns already have full rank; higher powers are unnecessary.
    assert np.linalg.matrix_rank(np.column_stack((b, a @ b))) == 6


def test_radial_holding_point_needs_feedforward_acceleration() -> None:
    a, b = hcw_state_space(N)
    desired = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    holding_acceleration = np.array([-3 * N**2 * 20.0, 0.0, 0.0])
    assert np.linalg.norm(a @ desired) > 0.0
    np.testing.assert_allclose(a @ desired + b @ holding_acceleration, 0.0, atol=1e-18)
    np.testing.assert_allclose(
        hcw_derivative(0.0, desired, N, holding_acceleration), 0.0, atol=1e-18,
    )


def test_radial_offset_produces_expected_drift_after_one_orbit() -> None:
    initial = np.array([200.0, -995.0, 0.0, 0.0, 0.0, 0.0])
    times = np.linspace(0.0, orbital_period(N), 301)
    final = propagate_uncontrolled(initial, times, N)[-1]
    expected = initial.copy()
    expected[1] -= 12 * np.pi * initial[0]
    np.testing.assert_allclose(final, expected, rtol=1e-9, atol=2e-6)


def test_no_secular_drift_condition_returns_after_one_orbit() -> None:
    initial = np.array([200.0, -995.0, 25.0, 0.03, -2 * N * 200.0, 0.015])
    times = np.linspace(0.0, orbital_period(N), 301)
    np.testing.assert_allclose(
        propagate_uncontrolled(initial, times, N)[-1], initial,
        rtol=1e-8, atol=2e-6,
    )


def test_integrator_initial_state_is_at_first_supplied_time() -> None:
    initial = np.array([100.0, -995.0, 25.0, 0.03, -0.01, 0.015])
    times = np.linspace(1200.0, 1800.0, 101)
    np.testing.assert_allclose(
        propagate_uncontrolled(initial, times, N),
        analytical_uncontrolled(initial, times - times[0], N),
        rtol=1e-8, atol=2e-6,
    )


@pytest.mark.parametrize("n", [0.0, -1.0, np.nan, np.inf])
def test_matrix_mean_motion_must_be_finite_and_positive(n: float) -> None:
    with pytest.raises(ValueError, match="finite and positive"):
        hcw_state_space(n)
