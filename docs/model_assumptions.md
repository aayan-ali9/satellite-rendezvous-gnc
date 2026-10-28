# Mathematical Model — Milestone 1

## Frames and units
We define a target-centered right-handed LVLH (local vertical / local horizontal) frame:
- **x** is radial, positive away from Earth.
- **y** is along-track, positive in the direction of the target's velocity.
- **z** is the orbit normal, `x cross y`.

The state is `[x, y, z, vx, vy, vz]` in meters and meters/second. The input is acceleration `[ax, ay, az]` in m/s^2. The target is in a circular 500 km orbit.

## Reference orbit
Mean motion: `n = sqrt(mu / r^3)` and `r = R_E + h`. Orbital period: `T = 2*pi/n`.

## HCW equations

    x_ddot = 3 n^2 x + 2 n y_dot + a_x
    y_ddot = -2 n x_dot + a_y
    z_ddot = -n^2 z + a_z

For Milestone 1, set all control accelerations to zero.

The velocities here are derivatives in the rotating LVLH frame, not inertial
velocity differences. See [the state-space derivation](hcw_state_space.md) for
the physical origins of each term, A and B, controllability, and the acceleration
required to hold a nonzero radial or cross-track waypoint.

## Limitations
- HCW equations are linearized near a **circular** reference orbit.
- Assumes point-mass Earth gravity, constant mean motion, and relatively small separation compared to orbit radius.
- Does not include perturbations, actuator dynamics, collisions, attitude, or navigation.
- Here a fixed nonzero radial offset without thrust generally leads to secular along-track drift.
- Matching a closed-form HCW result verifies the numerical implementation, **not** real-world physical fidelity. Later compare to nonlinear two-body truth.

## References
- NASA, *Dynamics of Earth orbiting formations*: https://ntrs.nasa.gov/citations/20060043838
- NASA, *Navigation Filter Best Practices*: https://ntrs.nasa.gov/search.jsp?R=20180003657
- SciPy solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
