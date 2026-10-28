
# LQR Controller Design

## 1. Objective

The controller guides a chaser spacecraft toward successive
relative-position waypoints while limiting available thrust
acceleration.

The controller is designed using linearized Hill-Clohessy-
Wiltshire (HCW) dynamics and evaluated using a separate,
nonlinear two-body orbital simulation.

## 2. Relative Motion Model

The target-centered LVLH state is

s = [x, y, z, vx, vy, vz]^T

where x is radial, y is along-track, and z is cross-track.

The HCW equations with control acceleration are:

    x_ddot = 3*n^2*x + 2*n*vy + ax
    y_ddot = -2*n*vx + ay
    z_ddot = -n^2*z + az

These equations are written in state-space form:

    s_dot = A*s + B*u

where u = [ax, ay, az]^T.

The A and B matrices are implemented in
`rendezvous/lqr.py`.

The target follows a circular reference orbit at
500 km altitude.

## 3. Linear Quadratic Regulator

For a desired waypoint state, define the tracking error:

    e = s - s_target

The requested feedback acceleration is:

    u_requested = -K*e

The LQR gain minimizes the infinite-horizon quadratic
cost for the unconstrained linear system:

    J = integral(e^T*Q*e + u^T*R*u) dt

The gain is obtained from the continuous-time algebraic
Riccati equation:

    A^T*P + P*A - P*B*R^-1*B^T*P + Q = 0

    K = R^-1*B^T*P

The implementation uses SciPy's
`solve_continuous_are` function.

## 4. Controller Weighting Matrices

The current implementation uses:

    Q = diag(1e-6, 1e-6, 1e-6,
             1e-2, 1e-2, 1e-2)

    R = 1e6 * I_3

Q penalizes relative-position and relative-velocity
errors, while R penalizes commanded acceleration.

These values form the controller's current design
configuration. They have not been established as
globally optimal through a systematic tuning study.

Because position, velocity, and acceleration have
different physical units, numerical weight magnitudes
should not be interpreted as dimensionless measures
of engineering importance.

## 5. Acceleration Saturation

Real spacecraft cannot generate unlimited acceleration.

The baseline simulation enforces:

    maximum acceleration = 0.001 m/s^2

If the requested acceleration magnitude exceeds this
limit, the command is scaled to preserve its direction:

    u_applied = u_requested * (
        a_max / ||u_requested||
    )

Otherwise, the requested command is applied unchanged.

This limits the magnitude of the complete three-axis
acceleration vector, rather than clipping each axis
independently.

Acceleration saturation means the implemented controller
is no longer the unconstrained optimal LQR solution.

## 6. Waypoint Guidance

The guidance sequence uses along-track waypoints:

    Stage 1: [0, -200, 0] m
    Stage 2: [0,  -50, 0] m
    Stage 3: [0,  -30, 0] m

The desired relative velocity at each waypoint is zero.

Intermediate waypoint transitions require:

    Position error <= 5.0 m
    Velocity error <= 0.10 m/s

These along-track stationary states are equilibria of
the ideal HCW model, so no nonzero linear-model holding
feedforward acceleration is required.

The final 30 m stage is propagated through the remaining
simulation duration, with final acceptance checked at
the end.

## 7. Nonlinear Simulation

Although the controller is designed using HCW dynamics,
the mission simulation propagates both spacecraft using
nonlinear two-body gravitational dynamics in ECI.

At each integration evaluation:

1. Convert the spacecraft states into relative LVLH.
2. Calculate the LQR tracking error.
3. Calculate the requested control acceleration.
4. Apply the acceleration magnitude limit.
5. Convert the applied acceleration from LVLH to ECI.
6. Propagate the spacecraft under gravity and control.

The applied acceleration magnitude is integrated over
time to estimate cumulative maneuver delta-v.

## 8. Validation and Limitations

The project includes automated unit and regression tests,
linear/nonlinear dynamics comparisons, and numerical
integration checks.

The nominal simulation produces approximately:

- Total maneuver delta-v: 1.026717 m/s
- Minimum separation: 29.520 m
- Final position error: 0.000533 m

These results demonstrate the implemented controller's
performance under idealized numerical assumptions.

Important limitations include:

- Perfect relative-state knowledge
- Continuous, perfectly directed acceleration
- No navigation measurement noise
- No J2, atmospheric drag, or third-body perturbations
- No discrete thruster or attitude-control dynamics
- No formal collision-avoidance guarantee

LQR optimality applies to the unconstrained linearized
design model, not automatically to the saturated,
nonlinear mission simulation.

## 9. Implementation References

- `rendezvous/lqr.py` - HCW matrices and LQR gain
- `rendezvous/control.py` - Acceleration saturation
- `rendezvous/nonlinear_control.py` - ECI dynamics and control
- `rendezvous/mission.py` - Mission configuration and guidance
- `tests/test_lqr.py` - Controller verification
- `tests/test_mission_regression.py` - Mission regression tests
