"""Physics and implementation checks for the Week 1 HCW baseline."""

import numpy as np
import pytest

from rendezvous.constants import circular_mean_motion, orbital_period
from rendezvous.hcw import (
    analytical_uncontrolled, hcw_derivative, propagate_uncontrolled,
)


N = circular_mean_motion(500_000.0)


def test_zero_state_remains_zero() -> None:
    initial = np.zeros(6)
    times = np.linspace(0.0, 6000.0, 100)
    np.testing.assert_allclose(propagate_uncontrolled(initial, times, N), 0.0, atol=1e-12)


def test_numerical_matches_closed_form() -> None:
    initial = np.array([100.0, -995.0, 25.0, 0.03, -0.01, 0.015])
    times = np.linspace(0.0, 2 * orbital_period(N), 500)
    numerical = propagate_uncontrolled(initial, times, N)
    analytical = analytical_uncontrolled(initial, times, N)
    np.testing.assert_allclose(numerical, analytical, rtol=1e-8, atol=2e-6)


def test_cross_track_returns_after_one_orbit() -> None:
    initial = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.02])
    one_period = orbital_period(N)
    final = analytical_uncontrolled(initial, np.array([one_period]), N)[0]
    np.testing.assert_allclose(final[[2, 5]], initial[[2, 5]], atol=1e-10)


def test_righthand_side_acceleration_input() -> None:
    state = np.zeros(6)
    accel = np.array([1e-4, -2e-4, 3e-4])
    derivative = hcw_derivative(0.0, state, N, accel)
    np.testing.assert_allclose(derivative[3:], accel)


def test_negative_altitude_rejected() -> None:
    with pytest.raises(ValueError):
        circular_mean_motion(-1.0)
