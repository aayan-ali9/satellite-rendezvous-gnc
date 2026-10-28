# HCW state-space milestone

This milestone expresses the existing equations as matrices. It introduces no new
orbital physics or controller. Read alongside `rendezvous/hcw.py` and
`tests/test_state_space.py`.

## Assumptions and coordinates

The target follows a circular two-body orbit. Earth is a point mass; mean motion
is constant. The chaser's displacement is small relative to the target's orbital
radius. We neglect perturbations, attitude, sensor errors, and actuator dynamics.
The three acceleration inputs are independently available in the target frame.
Controllability under this assumption does not establish feasibility with bounded
thrust or a real thruster arrangement.

Define relative displacement from target to chaser in a right-handed rotating
frame: x outward, y in the target's direction of flight, z along x cross y.
The velocities are derivatives of these rotating-frame coordinates. They are
not inertial velocity differences. Zero rotating-frame velocity initially means
the two spacecraft rotate together instantaneously; it does not imply that both
are independently following circular orbits at their respective radii.

## From circular motion to mean motion

Equate the target's centripetal and gravitational accelerations:

\[
n^2 r = \frac{\mu}{r^2},\qquad
n=\sqrt{\frac{\mu}{r^3}},\qquad r=R_E+h,\qquad T=\frac{2\pi}{n}.
\]

The code uses SI units: mu = 3.986004418e14 m^3/s^2, R_E = 6,378,137 m,
and h = 500,000 m. Use orbital radius, not altitude, in the denominator.

## What the acceleration terms mean

Linearizing the difference in gravity between the spacecraft produces
`n^2 * [2x, -y, -z]`. For the rotating frame with angular velocity
`omega = [0, 0, n]`, subtracting centrifugal-frame acceleration adds
`n^2 * [x, y, 0]`. Their sum is `[3n^2 x, 0, -n^2 z]`.
Subtracting `2 omega cross relative_velocity` contributes
`[2n vy, -2n vx, 0]`, the Coriolis terms. Commanded acceleration is then added.

Thus an outward x offset initially gives positive radial acceleration. Once vx
becomes positive, the along-track acceleration becomes negative. Motion behind
the target follows even with no commanded thrust. The cross-track equation is a
simple harmonic oscillator of angular frequency n.

MIT's derivation gives the same radial-first equations using xi, eta, zeta; it
also introduces a different x/y ordering later. Always map axes before copying
equations from a reference. See [MIT Lecture 26](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/resources/lec_26/).

## Build the matrices one row at a time

Use a bold s for the state to avoid confusing it with radial coordinate x:

\[
\mathbf{s}=[x,y,z,v_x,v_y,v_z]^T,\qquad
\mathbf{u}=[a_x,a_y,a_z]^T,\qquad
\dot{\mathbf{s}}=A\mathbf{s}+B\mathbf{u}.
\]

1. The first three derivatives are vx, vy, vz. Put ones in columns 4, 5, 6 of
   rows 1, 2, 3. These rows contain no direct acceleration input.
2. The fourth derivative is `3n^2 x + 2n vy + ax`. Put `3n^2` in column 1,
   `2n` in column 5, and a one in B's first input column.
3. The fifth derivative is `-2n vx + ay`. Put `-2n` in column 4, and a one in
   B's second input column.
4. The sixth derivative is `-n^2 z + az`. Put `-n^2` in column 3, and a one in
   B's third input column.

\[
A=\begin{bmatrix}
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&0&1\\
3n^2&0&0&0&2n&0\\
0&0&0&-2n&0&0\\
0&0&-n^2&0&0&0
\end{bmatrix},\qquad
B=\begin{bmatrix}
0&0&0\\0&0&0\\0&0&0\\1&0&0\\0&1&0\\0&0&1
\end{bmatrix}.
\]

Dimensional check: n^2 times meters and n times m/s both give m/s^2.
The state mixes positions and velocities, so entries of A have different
dimensions. B maps acceleration directly to velocity derivatives. If the input
were force in newtons, B's lower block would instead be I/mass.

In Python:

```python
from rendezvous.constants import circular_mean_motion
from rendezvous.hcw import hcw_state_space

n = circular_mean_motion(500_000.0)
A, B = hcw_state_space(n)
# derivative = A @ state + B @ acceleration
```

## Verification and controllability

We retain the scalar ODE as a separate implementation. Every state basis vector
and input basis vector is checked against the matrices. Next, `expm(A*t)` is
compared against the closed-form solution for all six initial-state basis vectors
at several times. Existing tests also compare numerical integration with that
solution. Orbital-period drift and periodic-motion tests check recognizable
physical cases, including a nonzero integration start time.

For blocks `A = [[0,I],[D,C]]` and `B = [[0],[I]]`,
`AB = [[I],[C]]`. The first six columns of the controllability matrix are
`[B, AB] = [[0,I],[I,C]]`. If these columns multiply `[p,q]` to zero, the upper
block gives q = 0 and the lower block gives p = 0. They are independent, so
the rank is six. No numerical rank calculation with many tiny powers of n is
needed to establish this result. The automated rank check is a supporting check.

These checks verify the linear model implementation. All three propagations
share HCW assumptions; agreement does not validate HCW against nonlinear orbital
motion. That remains a later, independent truth-model milestone.

## A holding waypoint must be an equilibrium

For fixed desired position `[xd, yd, zd]` and zero desired velocity, solve
`A @ desired_state + B @ holding_acceleration = 0`:

\[
\mathbf{u}_{hold}=[-3n^2x_d,\;0,\;n^2z_d]^T.
\]

A 20 m radial waypoint needs inward acceleration. A purely along-track waypoint
needs none in HCW. An out-of-plane waypoint needs acceleration away from the
orbital plane to oppose its restoring term. These are linear-model statements.

For future feedback `u = u_hold - K(s - s_desired)`, the error dynamics become
`error_dot = (A - BK) error` before saturation. Omitting u_hold generally leaves
a nonzero forcing term `A @ desired_state` and makes the desired state cease to
be an equilibrium. Acceleration saturation may also prevent the required hold.

## Next engineering decisions

Before LQR implementation, choose the holding-point axis and position tolerance,
acceleration bound, maneuver time allowance, and position/velocity scales used to
select Q and R. Verify any resulting controller first with equilibrium residuals,
closed-loop eigenvalues, and the unsaturated model; then measure performance
with saturation. Three-axis controllability alone guarantees none of these
mission requirements.

Learning exercise: identify each nonzero matrix entry without looking at the
code, then derive the holding acceleration for a waypoint of your own choice.
