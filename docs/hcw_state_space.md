# Hill–Clohessy–Wiltshire Relative Dynamics and State-Space Model

## Overview

The Hill–Clohessy–Wiltshire (HCW) equations provide a linearized representation of spacecraft relative motion about a circular reference orbit. In this project, the HCW model forms the mathematical basis for the Linear Quadratic Regulator (LQR) used to control a chaser spacecraft during autonomous rendezvous.

The model describes relative position and velocity in a target-centered Local Vertical Local Horizontal (LVLH) reference frame. Its state-space representation supports feedback controller design, analytical propagation, and numerical verification.

## 1. Model Assumptions

The HCW formulation assumes:

- The target spacecraft follows a circular reference orbit.
- Earth's gravitational field is represented by a central point-mass model.
- Chaser-target separation is small relative to the target's orbital radius.
- The reference orbit has constant mean motion.
- Control inputs represent commanded acceleration along three LVLH axes.

The LVLH coordinate convention is:

- **x:** Radially outward from Earth's center.
- **y:** Along the target's orbital velocity direction.
- **z:** Orbit-normal direction, completing a right-handed reference frame.

Relative velocity components are derivatives of position measured in the rotating LVLH frame, rather than simply differences between inertial velocity vectors.

### Reference Orbit

For a circular orbit, the mean motion is

$$
n=\sqrt{\frac{\mu}{r^3}}
$$

where

$$
r=R_E+h
$$

The project uses:

| Parameter | Value |
|---|---|
| Gravitational parameter, \(\mu\) | \(3.986004418\times10^{14}\ \mathrm{m^3/s^2}\) |
| Reference Earth radius, \(R_E\) | \(6,378,137\ \mathrm{m}\) |
| Reference altitude, \(h\) | \(500,000\ \mathrm{m}\) |

All calculations use SI units.

## 2. HCW Equations of Motion

The controlled HCW equations are

$$
\ddot{x}=3n^2x+2n\dot{y}+a_x
$$

$$
\ddot{y}=-2n\dot{x}+a_y
$$

$$
\ddot{z}=-n^2z+a_z
$$

where \(a_x\), \(a_y\), and \(a_z\) are commanded accelerations in the LVLH frame.

These equations capture three important effects:

**Gravity-gradient effects:** Differences in gravitational acceleration between the target and chaser contribute to the radial and cross-track relative dynamics.

**Rotating-frame effects:** Coriolis acceleration couples radial and along-track motion through terms proportional to the reference mean motion.

**Cross-track oscillation:** In the absence of control, cross-track relative motion follows a harmonic oscillator with angular frequency \(n\).

The radial and along-track equations are coupled, whereas cross-track motion is independent in the ideal linear model.

## 3. State-Space Representation

Define the six-dimensional relative state as

$$
\mathbf{s}=
\begin{bmatrix}
x & y & z & v_x & v_y & v_z
\end{bmatrix}^{T}
$$

and the control input as

$$
\mathbf{u}=
\begin{bmatrix}
a_x & a_y & a_z
\end{bmatrix}^{T}
$$

The continuous-time dynamics are expressed as

$$
\dot{\mathbf{s}}=A\mathbf{s}+B\mathbf{u}
$$

with

$$
A=
\begin{bmatrix}
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&0&1\\
3n^2&0&0&0&2n&0\\
0&0&0&-2n&0&0\\
0&0&-n^2&0&0&0
\end{bmatrix}
$$

and

$$
B=
\begin{bmatrix}
0&0&0\\
0&0&0\\
0&0&0\\
1&0&0\\
0&1&0\\
0&0&1
\end{bmatrix}.
$$

The upper three rows of \(A\) relate position derivatives to velocity. The lower three rows represent the relative accelerations predicted by the HCW equations.

The input matrix \(B\) maps commanded accelerations directly to the corresponding velocity derivatives. This formulation treats acceleration as the control input rather than spacecraft thrust force.

### Python Implementation

The HCW state-space matrices can be generated using:

```python
from rendezvous.constants import circular_mean_motion
from rendezvous.hcw import hcw_state_space

n = circular_mean_motion(500_000.0)
A, B = hcw_state_space(n)
```

The project implements this matrix formulation in `rendezvous/hcw.py` and also provides an equivalent matrix builder in `rendezvous/lqr.py`.

## 4. Controllability

For the linearized model, controllability describes whether the system can be driven between arbitrary states using the available control inputs, without imposing acceleration magnitude constraints.

The controllability matrix is

$$
\mathcal{C}=
\begin{bmatrix}
B&AB&A^2B&\cdots&A^5B
\end{bmatrix}.
$$

The HCW state-space matrices have block structure

$$
A=
\begin{bmatrix}
0&I\\
D&C
\end{bmatrix},
\qquad
B=
\begin{bmatrix}
0\\
I
\end{bmatrix}.
$$

Therefore,

$$
AB=
\begin{bmatrix}
I\\
C
\end{bmatrix}
$$

and the first six columns of the controllability matrix satisfy

$$
\operatorname{rank}
\begin{bmatrix}
B&AB
\end{bmatrix}=6.
$$

Consequently, the six-state HCW system is controllable when independent acceleration inputs are available along all three LVLH axes.

This result establishes controllability of the ideal linear system. It does not guarantee successful rendezvous under finite acceleration limits, safety constraints, or nonlinear orbital dynamics.

## 5. Stationary Relative Equilibria

For a constant desired relative position

$$
\mathbf{r}_d=
\begin{bmatrix}
x_d&y_d&z_d
\end{bmatrix}^{T}
$$

with zero relative velocity, the required holding acceleration must satisfy

$$
A\mathbf{s}_d+B\mathbf{u}_{hold}=0.
$$

Solving the HCW equilibrium equations gives

$$
\mathbf{u}_{hold}=
\begin{bmatrix}
-3n^2x_d\\
0\\
n^2z_d
\end{bmatrix}.
$$

A nonzero radial holding offset generally requires continuous acceleration in the ideal HCW model. Cross-track offsets also require holding acceleration.

A purely along-track stationary waypoint, with \(x_d=z_d=0\), is an equilibrium of the uncontrolled HCW model.

The project's final holding target is

$$
\mathbf{r}_d=
\begin{bmatrix}
0&-30&0
\end{bmatrix}^{T}\ \mathrm{m}.
$$

This along-track configuration requires no nonzero feedforward holding acceleration in the ideal HCW model, although corrections may still be needed in the nonlinear controlled simulation.

## 6. Verification

The HCW implementation is evaluated through independent numerical and analytical comparisons.

Verification methods include:

- Comparing the state-space derivative against the separately implemented scalar HCW equations.
- Comparing the matrix-exponential solution against the analytical HCW solution.
- Comparing numerical integration against analytical trajectories.
- Checking controllability through matrix rank.
- Evaluating orbital-period drift and periodic relative-motion cases.
- Verifying the equilibrium acceleration required for stationary holding positions.

Automated verification is implemented in `tests/test_state_space.py` and `tests/test_hcw.py`.

Agreement between analytical and numerical HCW solutions establishes consistency of the linear model implementation. It does not independently establish the accuracy of the linear approximation for arbitrary nonlinear spacecraft trajectories.

The project therefore also compares HCW predictions with a separate nonlinear two-body orbital propagation model.

## 7. Model Limitations

The HCW equations are most applicable when the reference orbit is approximately circular and relative separation remains small compared with orbital radius.

The linear model does not directly account for:

- Earth oblateness and J2 perturbations.
- Atmospheric drag or third-body gravitational effects.
- Navigation measurement error or state-estimation uncertainty.
- Finite thruster response, discrete firing, or spacecraft attitude dynamics.
- Nonlinear effects associated with large relative separations.

The HCW state-space formulation is used primarily for controller design and linear-model analysis. Closed-loop mission performance is evaluated separately using nonlinear two-body propagation and acceleration-limited feedback control.

## Implementation References

- `rendezvous/constants.py` — Reference-orbit quantities and mean motion.
- `rendezvous/hcw.py` — HCW dynamics, state-space matrices, and propagation.
- `rendezvous/lqr.py` — State-space construction and LQR gain calculation.
- `rendezvous/frames.py` — ECI/LVLH coordinate transformations.
- `rendezvous/nonlinear_control.py` — Nonlinear controlled orbital dynamics.
- `tests/test_state_space.py` — State-space and controllability verification.
- `tests/test_hcw.py` — HCW propagation verification.

### Reference

MIT OpenCourseWare, [16.346 Astrodynamics — Lecture 26](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/resources/lec_26/).