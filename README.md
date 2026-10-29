# Autonomous Satellite Rendezvous Using LQR Control

A Python simulation I developed to explore autonomous spacecraft rendezvous using orbital mechanics, feedback control, and numerical simulation.

The project models a chaser spacecraft approaching a target satellite in low Earth orbit. I used Hill–Clohessy–Wiltshire (HCW) equations to design an LQR controller, then tested its performance using a separate nonlinear two-body orbital simulation.

I also evaluated how the system responds to different initial conditions and available control accelerations.

## Mission Overview

The goal is to guide a chaser spacecraft from approximately 1 km away to a final holding position 30 m behind the target.

The mission uses three along-track waypoints:

| Stage | Desired position |
|---|---|
| 1 | 200 m behind the target |
| 2 | 50 m behind the target |
| 3 | 30 m behind the target (final hold) |

The controller continuously adjusts the chaser's acceleration to reduce its position and velocity errors relative to the current waypoint.

The simulation uses:

- A 500 km circular reference orbit.
- A baseline maximum acceleration of 0.001 m/s².
- A 5 m position tolerance and 0.10 m/s velocity tolerance for waypoint acceptance.
- A 25 m screening boundary monitored during the simulated approach.

### Simulated Rendezvous Trajectory

![Nonlinear Rendezvous Trajectory](figures/nonlinear_trajectory.png)

*Figure 1. Simulated three-waypoint approach shown in the target-centered LVLH frame.*

### Final Approach and Safety Boundaries

![Final Approach Safety](figures/final_approach_safety.png)

*Figure 2. Close-up of the final approach, including the 25 m screening boundary and 20 m keep-out reference.*

These figures show a simulated approach with a small initial velocity perturbation.

## How the Simulation Works

The project combines a linear controller with nonlinear orbital dynamics.

**1. Relative orbital motion**

I implemented the HCW equations to model how a spacecraft moves relative to another spacecraft in a circular reference orbit.

The equations are useful for controller design because they describe relative position and velocity in a target-centered LVLH coordinate system.

**2. LQR feedback control**

I used the HCW state-space model to implement a Linear Quadratic Regulator (LQR).

The controller compares the current relative state with the desired waypoint and calculates the acceleration needed to reduce the error.

The LQR gain is calculated using SciPy's algebraic Riccati equation solver.

**3. Nonlinear orbital simulation**

To test the controller beyond the linear HCW model, I implemented nonlinear two-body gravitational propagation for both spacecraft in Earth-Centered Inertial (ECI) coordinates.

During propagation, the simulation converts the spacecraft states into LVLH, calculates the control acceleration, applies the acceleration limit, and converts the command back to ECI.

**4. Autonomous waypoint transitions**

The chaser moves through the 200 m, 50 m, and 30 m waypoints. The first two transitions occur when both the position and velocity errors are within their allowed tolerances.

At the final waypoint, the simulation continues through the remaining mission time to evaluate holding performance.

**5. Performance evaluation**

The simulation records separation distance, final position and velocity errors, and estimated maneuver delta-v.

I estimate maneuver delta-v by numerically integrating the magnitude of the applied control acceleration over time.

For more detail, see the [HCW State-Space Model](docs/hcw_state_space.md) and [LQR Controller Design](docs/lqr_controller_design.md).

## Simulation Results

The unperturbed baseline mission produced the following results:

| Metric | Result |
|---|---|
| Final holding distance | 30 m |
| Minimum separation | Approximately 29.52 m |
| Total maneuver delta-v | Approximately 1.027 m/s |
| Final position error | Approximately 0.00053 m |
| Final velocity error | Approximately 0.0000011 m/s |
| Maximum available acceleration | 0.001 m/s² |
| Final waypoint acceptance | PASS |

These results are from an idealized numerical model with perfect relative-state information. The very small final errors reflect the simulation assumptions and should not be interpreted as achievable real-world navigation accuracy.

## Initial-Condition and Sensitivity Analysis

After completing the baseline mission, I tested how the controller performed with changes to the chaser's initial position and velocity.

I ran 52 reproducible test scenarios with different levels of initial-state perturbation.

- **51 scenarios completed successfully.**
- **1 scenario failed to reach the first waypoint within the simulation time.**

I then investigated the failed scenario by testing different maximum control accelerations.

For this particular case, the simulation failed at 0.001550 m/s² but succeeded at 0.001575 m/s².

This showed how the available acceleration affected whether the controller could complete the mission under the selected conditions. The results do not establish a universal minimum acceleration requirement or real-world mission reliability.

### Mission Success vs. Available Acceleration

![Acceleration Sensitivity](figures/acceleration_success.png)

*Figure 3. Simulated success and failure outcomes for the challenging initial condition at different acceleration limits.*

### Maneuver Delta-v vs. Available Acceleration

![Acceleration Delta V](figures/acceleration_delta_v.png)

*Figure 4. Estimated maneuver delta-v for successful runs in the acceleration sensitivity study.*

### Final Holding Performance

![Final Holding Separation](figures/final_holding_separation.png)

*Figure 5. Spacecraft separation during the simulated final holding phase.*

## Project Structure

| Directory | Contents |
|---|---|
| `rendezvous/` | Orbital dynamics, coordinate transformations, guidance, and control |
| `scripts/` | Mission simulations, model comparisons, and sensitivity studies |
| `tests/` | Automated tests for the simulation and controller |
| `figures/` | Trajectory and performance plots |
| `results/` | Saved simulation results |
| `docs/` | Technical explanations of the HCW model and LQR controller |

The reusable mission simulation is implemented in `rendezvous/mission.py`, while `rendezvous/nonlinear_control.py` handles the controlled nonlinear equations of motion.

## Running the Project

The project uses Python with NumPy, SciPy, Matplotlib, and pytest.

Install the required packages:

```powershell
python -m pip install -r requirements.txt
```

Run the nonlinear three-waypoint simulation:

```powershell
python -m scripts.run_nonlinear_three_waypoints
```

Run the initial-condition study:

```powershell
python -m scripts.run_robustness_sweep
```

Run the acceleration sensitivity study:

```powershell
python -m scripts.run_acceleration_study
```

Generate the acceleration sensitivity figures from the saved sweep datasets:

```powershell
python -m scripts.plot_acceleration_sweep
```

Run the automated tests:

```powershell
python -m pytest -q -p no:cacheprovider
```

## Assumptions and Limitations

This is an engineering simulation, not flight-ready rendezvous software.

The main simplifying assumptions are:

- The target begins in a circular reference orbit.
- Gravity is modeled using the two-body approximation.
- Control acceleration is continuous and can be applied in any direction.
- Relative position and velocity are assumed to be known exactly.
- Navigation sensor noise and state-estimation errors are not modeled.
- Orbital perturbations such as J2 and atmospheric drag are not included.
- The separation checks do not provide a formal collision-avoidance guarantee.

The project focuses on implementing and testing a preliminary spacecraft guidance and control system under these assumptions.

## Tools

**Python, NumPy, SciPy, Matplotlib, pytest**

**Engineering topics:** Orbital mechanics, HCW relative motion, LQR control, nonlinear simulation, coordinate transformations, and sensitivity analysis.