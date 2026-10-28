# Mission Requirements — working draft

## Mission definition
A chaser spacecraft in low Earth orbit must use an autonomous guidance, navigation, and control (GN&C) loop to approach a passive target and establish a low-relative-velocity hold at a designated stand-off point.

## Reference scenario (provisional)
- Target circular orbital altitude: 500 km
- Initial separation: approximately 1 km
- Target holding point: 20 m away from target center (direction to be chosen and justified)
- Terminal relative speed: < 0.05 m/s
- Dynamics baseline: circular-chief Hill–Clohessy–Wiltshire equations
- All core code and plots use SI units

## Verification metrics
| ID | Requirement | Verification | Status |
| --- | --- | --- | --- |
| DYN-01 | HCW numerical propagation matches closed-form free-motion solution | Automated test | Implemented |
| DYN-02 | HCW deviations from nonlinear two-body truth are quantified | Error versus time and separation | Planned |
| GNC-01 | Chaser reaches a 20 m standoff waypoint | Terminal position/range test | Planned |
| GNC-02 | Terminal relative speed is < 0.05 m/s | Terminal speed test | Planned |
| GNC-03 | Actuator commands respect a documented acceleration bound | Time-history check | Planned |
| NAV-01 | Estimated navigation state is evaluated versus known truth | RMS error and consistency | Planned |
| VAL-01 | Monte Carlo test results are reproducible with fixed seeds | Versioned script and metrics | Planned |
| SAFE-01 | Chaser respects defined keep-out constraints | Trajectory intersection test | Future extension |

**Important:** A successful 20 m hold is not docking. Keep-out-zone logic and abort/retreat behavior are separate capabilities and should not be claimed until implemented.

## Engineering decisions to make
1. Define exact target waypoint in the target LVLH frame (e.g., a point along the radial or along-track axis).
2. Choose max acceleration and initial spacecraft mass for plausible thruster sizing.
3. Specify allowable position error and maximum maneuver duration.
4. Set sensor sample rate, measurement type, noise, and failure assumptions.
5. Define target-state knowledge assumptions, collision/keep-out rules, and whether control is continuous or discretized.
