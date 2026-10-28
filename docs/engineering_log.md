# Engineering Log

## Decision 001 — Initial dynamics model
**Question:** Which dynamics model should be used for the first prototype?

**Decision:** HCW relative dynamics in target LVLH at 500 km circular reference orbit.

**Reason:** It provides a tractable 6-state, linear dynamics model for verifying coordinate frames, control logic, and navigation assumptions.

**Consequence:** It is not a high-fidelity truth model; nonlinear two-body validation is explicitly planned.

## Experiment entry template
- **Date / code commit:**
- **Question or hypothesis:**
- **Initial conditions / units:**
- **Method and reference:**
- **Plots / numerical results:**
- **Does evidence support hypothesis?:**
- **Limitations / follow-up:**

## Experiment 002 — Baseline audit and initial-condition sensitivity

- **Date:** October 13, 2026; no local Git commit exists yet.
- **Question:** Is the Week 1 implementation consistent with analytical HCW,
  and why does increasing the radial offset increase separation?
- **Method:** Preserved the existing 200 m radial-offset experiment; ran its
  original five tests, then the expanded 16-test suite. Compared numerical and
  analytical propagation for two periods at a circular target altitude of 500 km.
  Compared zero-thrust cases with x0 = 100 m, x0 = 200 m, and x0 = 200 m with
  vy0 = -2 n x0; all use y0 = -995 m and zero remaining components.
- **Results:** All 16 tests passed. Baseline final range was 16,075.889 m.
  Maximum sampled numerical/analytical differences were 4.477413e-8 m and
  5.302043e-11 m/s. The 100 m case ended at 8,535.408 m; the drift-canceling
  case returned to its initial state, with range 1,014.901 m.
- **Interpretation:** The secular y term is proportional to 6 n x0 + 3 vy0.
  A chosen initial velocity cancels it; zero thrust alone does not imply
  constant separation.
- **Artifacts:** `results/baseline_summary.json`,
  `results/initial_condition_summary.json`, the three PNG plots, and
  `docs/week1_review.md`. Resolved dependencies are in `requirements-lock.txt`.
- **Limitations / follow-up:** This verifies HCW implementation only. Choose
  holding-point geometry and acceleration bounds before LQR; compare against
  nonlinear truth dynamics in a later milestone.
