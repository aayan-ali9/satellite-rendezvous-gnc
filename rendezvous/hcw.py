"""Hill-Clohessy-Wiltshire (HCW) relative-motion model.

Target-centered LVLH axes:
  x: radially outward from Earth
  y: along the target's direction of flight
  z: orbit-normal, completing a right-handed coordinate system

State: [x, y, z, vx, vy, vz] in meters and meters/second.
Control: [ax, ay, az] in meters/second^2.
Assumptions: circular target orbit, near-target relative motion, point-mass gravity.
"""

import numpy as np
from scipy.integrate import solve_ivp


def hcw_state_space(n: float) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous-time A (6x6) and B (6x3) in the documented LVLH axes.

    State: [x, y, z, vx, vy, vz]; input: commanded acceleration in m/s^2.
    This is the same linear physics as hcw_derivative, not a controller.
    """
    if not np.isfinite(n) or n <= 0:
        raise ValueError("Mean motion must be finite and positive")
    a = np.zeros((6, 6))
    a[:3, 3:] = np.eye(3)
    a[3, 0] = 3.0 * n**2
    a[3, 4] = 2.0 * n
    a[4, 3] = -2.0 * n
    a[5, 2] = -n**2
    b = np.zeros((6, 3))
    b[3:, :] = np.eye(3)
    return a, b


def hcw_derivative(t: float, state: np.ndarray, n: float,
                   acceleration: np.ndarray | None = None) -> np.ndarray:
    """Derivative of the six-state HCW model. ``t`` is supplied for ODE APIs."""
    _ = t
    x, y, z, vx, vy, vz = np.asarray(state, dtype=float)
    if n <= 0:
        raise ValueError("Mean motion must be positive")
    u = np.zeros(3) if acceleration is None else np.asarray(acceleration, dtype=float)
    if u.shape != (3,):
        raise ValueError("Acceleration must be a length-3 vector")
    return np.array([
        vx,
        vy,
        vz,
        3.0 * n**2 * x + 2.0 * n * vy + u[0],
        -2.0 * n * vx + u[1],
        -n**2 * z + u[2],
    ])


def propagate_uncontrolled(initial_state: np.ndarray,
                           times_s: np.ndarray, n: float) -> np.ndarray:
    """Numerically integrate uncontrolled HCW over increasing time points.

    Returns an array with shape (number_of_times, 6).
    """
    initial_state = np.asarray(initial_state, dtype=float)
    times_s = np.asarray(times_s, dtype=float)
    if initial_state.shape != (6,):
        raise ValueError("Initial state must contain six values")
    if times_s.ndim != 1 or len(times_s) < 2 or np.any(np.diff(times_s) <= 0):
        raise ValueError("Times must be a strictly increasing 1D array")
    solution = solve_ivp(
        hcw_derivative,
        (float(times_s[0]), float(times_s[-1])),
        initial_state,
        t_eval=times_s,
        args=(n,),
        rtol=1e-11,
        atol=1e-12,
    )
    if not solution.success:
        raise RuntimeError(f"HCW integration failed: {solution.message}")
    return solution.y.T


def analytical_uncontrolled(initial_state: np.ndarray, times_s: np.ndarray,
                            n: float) -> np.ndarray:
    """Closed-form HCW solution for independent numerical verification.

    ``times_s`` represents elapsed time since initial_state, not absolute epoch.
    """
    x0, y0, z0, vx0, vy0, vz0 = np.asarray(initial_state, dtype=float)
    times_s = np.atleast_1d(np.asarray(times_s, dtype=float))
    if n <= 0:
        raise ValueError("Mean motion must be positive")
    nt = n * times_s
    c, s = np.cos(nt), np.sin(nt)
    x = (4 - 3*c)*x0 + (s/n)*vx0 + (2*(1-c)/n)*vy0
    y = 6*(s-nt)*x0 + y0 - (2*(1-c)/n)*vx0 + (4*s/n-3*times_s)*vy0
    z = c*z0 + (s/n)*vz0
    vx = 3*n*s*x0 + c*vx0 + 2*s*vy0
    vy = 6*n*(c-1)*x0 - 2*s*vx0 + (4*c-3)*vy0
    vz = -n*s*z0 + c*vz0
    return np.column_stack((x, y, z, vx, vy, vz))
