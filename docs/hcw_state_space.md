# HCW Relative Motion and State-Space Model

## Overview

I used the Hill–Clohessy–Wiltshire (HCW) equations to model the relative motion between a target satellite and a chaser spacecraft. HCW provides a simpler way to describe how two nearby spacecraft move relative to each other without having to propagate both orbits separately.

For this project, I used HCW dynamics to build the state-space model for an LQR controller. I then tested the controller using a separate nonlinear two-body simulation.

## 1. Assumptions and Reference Frame

The HCW model makes several simplifying assumptions:

- The target follows a circular orbit around Earth.
- The chaser stays relatively close to the target compared with the orbital radius.
- Earth's gravity is modeled using the two-body approximation.
- Other effects, such as atmospheric drag and J2 perturbations, are neglected.

The simulation uses a target-centered Local Vertical Local Horizontal (LVLH) reference frame:

- **x:** Radially outward from Earth.
- **y:** Along the target's direction of motion.
- **z:** Perpendicular to the orbital plane.

Using LVLH makes it easier to describe the chaser's motion in terms of its distance and velocity relative to the target.

The reference orbit is at an altitude of 500 km. Its mean motion is calculated using

$$
n = \sqrt{\frac{\mu}{r^3}}
$$

where \(\mu\) is Earth's gravitational parameter and \(r\) is the orbital radius measured from Earth's center.

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

Here, \(n\) is the target's mean motion and \(a_x\), \(a_y\), and \(a_z\) are the control accelerations applied by the chaser.

There are a few important things to understand about these equations.

First, radial and along-track motion are coupled. Motion in one direction can affect acceleration in the other because the LVLH frame rotates with the target's orbit.

Second, the \(3n^2x\) term represents part of the gravity-gradient effect. A chaser displaced radially from the target experiences slightly different gravitational acceleration.

Finally, the cross-track equation is independent of the other two axes in the linear HCW model. Without control acceleration, it behaves like a harmonic oscillator.

These effects explain why controlling spacecraft relative motion is more complicated than simply moving an object from one point to another in a stationary reference frame.

## 3. State-Space Representation

To design the LQR controller, I expressed the HCW equations in state-space form:

$$
\dot{\mathbf{s}}=A\mathbf{s}+B\mathbf{u}
$$

The state vector contains the chaser's relative position and velocity:

$$
\mathbf{s}=
\begin{bmatrix}
x&y&z&v_x&v_y&v_z
\end{bmatrix}^{T}
$$

The input vector contains the three commanded accelerations:

$$
\mathbf{u}=
\begin{bmatrix}
a_x&a_y&a_z
\end{bmatrix}^{T}
$$

The resulting matrices are

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

$$
B=
\begin{bmatrix}
0&0&0\\
0&0&0\\
0&0&0\\
1&0&0\\
0&1&0\\
0&0&1
\end{bmatrix}
$$

The first three rows of \(A\) describe how position changes with velocity. The last three rows describe the HCW accelerations.

The \(B\) matrix shows that the control inputs directly affect acceleration, which changes velocity and eventually position.

This form is useful because the LQR controller can use the matrices \(A\) and \(B\) to calculate a feedback gain.

## 4. Python Implementation

I implemented the equations in `rendezvous/hcw.py`, including:

- A function that calculates the HCW state derivative.
- A function that constructs the state-space matrices.
- A numerical propagator using SciPy's `solve_ivp`.
- An analytical HCW solution for comparison.

The main LQR implementation is in `rendezvous/lqr.py`.

One advantage of keeping the dynamics and controller calculations in separate functions is that I could test the orbital model before adding closed-loop control.

## 5. Verification

Before using HCW for controller design, I checked that the implementation gave consistent results.

I compared the numerical integration with the analytical HCW solution and checked that the state-space matrices produced the same derivatives as the original equations.

I also checked the controllability of the linear system. With independent acceleration inputs along all three LVLH axes, the model has a full-rank controllability matrix.

These tests helped verify that the equations were implemented correctly.

However, agreement between different HCW calculations does not mean the model perfectly describes real spacecraft motion. HCW is still a linear approximation.

For that reason, the final rendezvous simulation uses nonlinear two-body propagation to evaluate the controller.

## 6. How HCW Fits Into the Rendezvous Mission

The mission uses three along-track waypoints at approximately 200 m, 50 m, and 30 m behind the target.

The HCW model is used to design the LQR feedback controller. The controller then calculates the acceleration needed to reduce the difference between the chaser's current relative state and the desired waypoint state.

An interesting feature of HCW is that a purely along-track position with zero relative velocity is an equilibrium of the ideal linear model. This makes an along-track holding position useful for the mission.

The actual simulation still evaluates the spacecraft using nonlinear dynamics and limits the acceleration that the chaser can apply.

## 7. Limitations

The HCW model is useful for preliminary controller design, but it has limitations.

It assumes a circular reference orbit, small relative separation, and simplified gravitational dynamics. The project also assumes perfect relative-state information and idealized thrust acceleration.

The results therefore demonstrate the behavior of the simulated guidance and control system under these assumptions, rather than flight-qualified rendezvous performance.

## Related Files

- `rendezvous/constants.py` — Orbital constants and mean motion.
- `rendezvous/hcw.py` — HCW equations and propagation.
- `rendezvous/lqr.py` — State-space matrices and LQR gain.
- `rendezvous/nonlinear_control.py` — Nonlinear controlled dynamics.
- `tests/test_hcw.py` — HCW verification tests.
- `tests/test_state_space.py` — State-space and controllability tests.

## Reference

MIT OpenCourseWare, [16.346 Astrodynamics — Lecture 26](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/resources/lec_26/).