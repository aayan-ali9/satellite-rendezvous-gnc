
"""Discrete HCW Kalman filter for noisy relative-position measurements.

State: [x, y, z, vx, vy, vz] in target-centered LVLH.
Measurements: [x, y, z] in meters. No velocity sensor is assumed.
"""

import numpy as np
from scipy.linalg import expm

from rendezvous.hcw import hcw_state_space


def discrete_hcw_model(mean_motion: float, dt: float):
    """Return exact zero-order-hold state and acceleration-input matrices.

    Assumes the acceleration stays constant during each sample interval.
    """
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")

    A, B = hcw_state_space(mean_motion)
    augmented = np.zeros((9, 9))
    augmented[:6, :6] = A
    augmented[:6, 6:] = B
    transition = expm(augmented * dt)
    return transition[:6, :6], transition[:6, 6:]


class HCWKalmanFilter:
    """Predict six-state relative motion and update with noisy 3D positions."""

    def __init__(
        self,
        mean_motion: float,
        dt: float,
        initial_state: np.ndarray,
        initial_covariance: np.ndarray,
        position_noise_std: float = 5.0,
        acceleration_noise_std: float = 1e-5,
    ):
        if not np.isfinite(position_noise_std) or position_noise_std <= 0:
            raise ValueError("position_noise_std must be finite and positive")
        if not np.isfinite(acceleration_noise_std) or acceleration_noise_std < 0:
            raise ValueError("acceleration_noise_std must be finite and nonnegative")

        self.state = np.asarray(initial_state, dtype=float).copy()
        self.covariance = np.asarray(initial_covariance, dtype=float).copy()
        if self.state.shape != (6,) or self.covariance.shape != (6, 6):
            raise ValueError("Expected a six-state vector and 6x6 covariance")
        if not (np.all(np.isfinite(self.state)) and np.all(np.isfinite(self.covariance))):
            raise ValueError("State and covariance must be finite")

        self.F, self.G = discrete_hcw_model(mean_motion, dt)

        # Unknown constant acceleration during each interval is process noise.
        self.Q = acceleration_noise_std**2 * (self.G @ self.G.T)
        self.H = np.hstack((np.eye(3), np.zeros((3, 3))))
        self.R = position_noise_std**2 * np.eye(3)

    def predict(self, applied_acceleration: np.ndarray):
        """Predict using the applied (saturated) LVLH control command."""
        control = np.asarray(applied_acceleration, dtype=float)
        if control.shape != (3,):
            raise ValueError("Applied acceleration must contain three values")

        self.state = self.F @ self.state + self.G @ control
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        return self.state.copy()

    def update(self, measured_position: np.ndarray):
        """Update the estimate using a relative-position measurement."""
        measurement = np.asarray(measured_position, dtype=float)
        if measurement.shape != (3,):
            raise ValueError("Measurement must contain three positions")

        residual = measurement - self.H @ self.state
        innovation_cov = self.H @ self.covariance @ self.H.T + self.R
        gain = np.linalg.solve(
            innovation_cov, self.H @ self.covariance
        ).T

        self.state = self.state + gain @ residual

        # Joseph covariance form avoids losing symmetry from roundoff.
        identity_minus_kh = np.eye(6) - gain @ self.H
        self.covariance = (
            identity_minus_kh @ self.covariance @ identity_minus_kh.T
            + gain @ self.R @ gain.T
        )
        return self.state.copy()
