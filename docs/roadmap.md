# Development roadmap

## Milestone 1 — Verified free motion (starter includes this baseline)
- Understand target LVLH axes and state definition.
- Implement HCW ODE in `rendezvous/hcw.py`.
- Simulate uncontrolled motion in `scripts/run_baseline.py`.
- Compare numerical integration with the closed-form HCW solution.
- Describe results physically; change initial-state components and predict their effects.

## Milestone 2 — Guidance & control
- Write state-space form: x_dot = A x + B u.
- Confirm controllability; choose control units and maximum acceleration.
- Use a waypoint reference and LQR regulator (or carefully chosen feedforward-plus-feedback).
- Account for acceleration saturation and solve steady-state offset as necessary.
- Measure time-to-arrival, terminal errors, and approximate delta-v = integral ||u|| dt.

## Milestone 3 — Navigation
- Specify physically meaningful observation model (start with relative position measurements).
- Use a **linear** Kalman filter for HCW before considering EKF for nonlinear dynamics.
- Separate truth state, simulated sensor measurement, and navigation estimate.
- Compare estimated-state feedback with perfect-state feedback.
- Plot RMS errors, covariance bounds, and innovations; discuss assumptions and observability.

## Milestone 4 — Independent truth and uncertainty
- Integrate the nonlinear two-body acceleration for each spacecraft in an inertial frame.
- Transform the difference into target LVLH for comparison.
- Validate frame definitions, units, and transform consistency.
- Sweep initial separations and time horizons; plot HCW versus truth error.
- Run Monte Carlo with fixed seeds; publish failure cases, not only successes.

## Milestone 5 — Professional deliverables
- Short technical report (problem, model, design, validation, results, limitations).
- One GIF or short video of the closed-loop approach.
- Figures: trajectory, range/speed, commanded acceleration, delta-v, navigation errors, MC distributions.
- Automated regression tests via CI and a reproducible one-command demo.
- Brief portfolio summary with demonstrated numbers (do not invent results).

## Stretch goals (only after core is validated)
- J2 perturbations, drag, elliptical target orbits.
- Line-of-sight or range/range-rate measurement model and EKF.
- State/control constraints, keep-out-zone safety logic, approach corridor, abort maneuver.
- Compare LQR with constrained MPC, including computation time and tradeoffs.
- STK or Basilisk cross-validation, not a dependency of the core model.
