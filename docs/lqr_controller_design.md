# LQR Controller Design

## Overview

I used a Linear Quadratic Regulator (LQR) to control the chaser spacecraft during the rendezvous simulation.

The goal of the controller is to reduce the chaser's position and velocity errors relative to a desired waypoint while avoiding unnecessarily large acceleration commands.

I designed the controller using the linear HCW equations and tested it using nonlinear two-body orbital propagation.

## 1. State-Space Model

LQR uses a linear state-space model of the form

$$
\dot{\mathbf{s}}=A\mathbf{s}+B\mathbf{u}
$$

For this project, the state vector contains the chaser's relative position and velocity in the LVLH reference frame:

$$
\mathbf{s}=
\begin{bmatrix}
x&y&z&v_x&v_y&v_z
\end{bmatrix}^{T}
$$

The control vector contains the commanded acceleration along each LVLH axis:

$$
\mathbf{u}=
\begin{bmatrix}
a_x&a_y&a_z
\end{bmatrix}^{T}
$$

I constructed the state-space matrices from the HCW equations. The implementation is in `rendezvous/lqr.py`.

This lets me use a standard linear control method to calculate the acceleration needed to correct the spacecraft's relative motion.

## 2. LQR Feedback Control

The controller compares the current state with the desired waypoint state.

The tracking error is

$$
\mathbf{e}=\mathbf{s}-\mathbf{s}_{desired}
$$

The requested acceleration is calculated using

$$
\mathbf{u}_{requested}=-K\mathbf{e}
$$

where \(K\) is the LQR feedback gain matrix.

The negative sign provides feedback that acts to reduce the state error.

LQR calculates the gain by balancing two objectives:

- Reducing the spacecraft's position and velocity errors.
- Avoiding unnecessarily large control accelerations.

The mathematical cost function is

$$
J=\int_0^\infty
\left(
\mathbf{e}^{T}Q\mathbf{e}
+
\mathbf{u}^{T}R\mathbf{u}
\right)dt
$$

Here, \(Q\) weights the state errors and \(R\) weights the control effort.

I used SciPy's `solve_continuous_are` function to solve the continuous-time algebraic Riccati equation and then calculate the feedback gain.

The gain is

$$
K=R^{-1}B^TP
$$

where \(P\) is the solution to the Riccati equation.

I did not implement a Riccati equation solver from scratch. My implementation builds the spacecraft state-space model, specifies the controller weights, computes the gain using SciPy, and applies that gain during the simulation.

## 3. Controller Weights

The controller uses the following matrices:

$$
Q=\operatorname{diag}
\left(
10^{-6},10^{-6},10^{-6},
10^{-2},10^{-2},10^{-2}
\right)
$$

$$
R=10^6I_3
$$

The first three entries of \(Q\) penalize relative-position errors, while the last three penalize relative-velocity errors.

The \(R\) matrix penalizes commanded acceleration.

Changing these matrices affects the controller's response. For example, increasing the penalty on control effort generally produces a less aggressive unconstrained controller, while increasing the state-error penalties generally encourages faster correction.

However, these relationships are more complicated once acceleration limits and nonlinear dynamics are included.

The selected weights form the controller configuration used in this project. I did not perform a comprehensive optimization to prove that these values are the best possible choice.

## 4. Acceleration Limits

One important part of the implementation is that the controller cannot apply unlimited acceleration.

For the baseline mission, I used a maximum acceleration magnitude of

$$
a_{max}=0.001\ \mathrm{m/s^2}
$$

The LQR controller first calculates the acceleration it would like to apply.

If the requested acceleration exceeds the maximum magnitude, the command is scaled down while preserving its direction.

For a requested vector above the limit,

$$
\mathbf{u}_{applied}
=
\mathbf{u}_{requested}
\frac{a_{max}}
{\|\mathbf{u}_{requested}\|}
$$

Otherwise, the requested acceleration is applied without modification.

This is implemented in `rendezvous/control.py`.

An important distinction is that LQR gives the optimal feedback gain for the selected linear model and quadratic cost without this acceleration constraint. Once the acceleration is limited, the complete controller is no longer guaranteed to have the same optimality properties.

## 5. Waypoint Guidance

The mission uses three target positions in LVLH:

| Stage | Desired relative position |
|---|---|
| 1 | [0, −200, 0] m |
| 2 | [0, −50, 0] m |
| 3 | [0, −30, 0] m |

The desired relative velocity at each waypoint is zero.

The first two waypoint transitions require:

- Position error no greater than 5 m.
- Velocity error no greater than 0.10 m/s.

Both conditions must be satisfied before the controller transitions to the next waypoint.

During the last stage, the simulation continues through the remaining mission time and evaluates the final position and velocity errors.

I selected along-track holding positions because these are stationary equilibria in the ideal HCW model when the radial and cross-track offsets are zero.

## 6. Combining LQR with Nonlinear Dynamics

Although the controller is designed using HCW equations, the final simulation does not propagate the spacecraft using HCW alone.

Instead, I calculate the motion of both spacecraft using nonlinear two-body gravity in ECI coordinates.

During each evaluation of the nonlinear dynamics:

1. The target and chaser positions and velocities are represented in ECI.
2. The simulation converts them into a relative LVLH state.
3. The controller calculates the position and velocity errors relative to the current waypoint.
4. The LQR gain produces a requested acceleration.
5. The acceleration limit is applied.
6. The applied acceleration is transformed from LVLH to ECI.
7. The target and chaser states are propagated using nonlinear two-body gravity, with control acceleration applied only to the chaser.

The nonlinear controlled dynamics are implemented in `rendezvous/nonlinear_control.py`.

This arrangement allowed me to design the controller using a simpler linear model while evaluating its behavior under more realistic orbital equations.

## 7. Measuring Performance

The main performance metrics are:

- Final position error.
- Final relative velocity error.
- Minimum separation from the target.
- Estimated total maneuver delta-v.
- Whether the mission completed its waypoint sequence.

The maneuver delta-v is estimated from

$$
\Delta v=\int_0^{t_f}
\|\mathbf{u}_{applied}(t)\|dt
$$

I used numerical trapezoidal integration to estimate this quantity from the simulated acceleration history.

This represents accumulated control acceleration expressed as an equivalent delta-v. It is not a detailed propellant-consumption calculation.

For the unperturbed baseline simulation, the estimated total delta-v was approximately 1.027 m/s, with a minimum separation of approximately 29.52 m.

## 8. Testing and Limitations

I tested the controller using automated Python tests, numerical comparisons, and simulations with different initial conditions.

I also ran a sensitivity study to investigate a challenging case where the baseline acceleration limit was insufficient to reach the first waypoint.

By increasing the available acceleration, I observed that the same initial condition could complete the mission.

These results helped me understand the relationship between control authority and mission performance for the tested conditions.

However, the project still makes several simplifying assumptions:

- The controller receives exact relative-position and velocity information.
- The spacecraft can produce continuous acceleration in any direction.
- Orbital perturbations and navigation sensor errors are not included.
- The simulation does not model individual thrusters or attitude-control dynamics.
- The screening-boundary check is not a formal collision-avoidance guarantee.

The results demonstrate the implemented controller's behavior within the simulated conditions, not guaranteed performance for an actual spacecraft.

## Related Files

- `rendezvous/lqr.py` — LQR gain and state-space model.
- `rendezvous/control.py` — Acceleration limiting.
- `rendezvous/guidance.py` — Waypoint acceptance.
- `rendezvous/nonlinear_control.py` — Controlled nonlinear dynamics.
- `rendezvous/mission.py` — Mission sequence and performance calculations.
- `tests/test_lqr.py` — LQR controller tests.
- `tests/test_mission_regression.py` — Mission regression tests.