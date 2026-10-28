
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.twobody import MU_EARTH, two_body_derivative
from rendezvous.frames import lvlh_to_eci, eci_to_lvlh
from rendezvous.lqr import hcw_state_space


# 1. Initialize a circular target orbit
radius = 6_878_000.0
speed = np.sqrt(MU_EARTH / radius)
n = np.sqrt(MU_EARTH / radius**3)
period = 2 * np.pi / n

r_target = np.array([radius, 0.0, 0.0])
v_target = np.array([0.0, speed, 0.0])

target_initial = np.concatenate((r_target, v_target))


# 2. Define identical relative initial conditions
relative_initial = np.array([
    100.0, 0.0, 0.0,
    0.0, 0.0, 0.0
])

r_chaser, v_chaser = lvlh_to_eci(
    r_target,
    v_target,
    relative_initial[:3],
    relative_initial[3:],
    n
)

chaser_initial = np.concatenate((r_chaser, v_chaser))


# 3. Propagate both models over the same time interval
times = np.linspace(0.0, period, 500)

A, B = hcw_state_space(n)

hcw = solve_ivp(
    lambda t, state: A @ state,
    (0.0, period),
    relative_initial,
    t_eval=times,
    rtol=1e-10,
    atol=1e-10,
    max_step=30.0
)

target = solve_ivp(
    two_body_derivative,
    (0.0, period),
    target_initial,
    t_eval=times,
    rtol=1e-10,
    atol=1e-10,
    max_step=30.0
)

chaser = solve_ivp(
    two_body_derivative,
    (0.0, period),
    chaser_initial,
    t_eval=times,
    rtol=1e-10,
    atol=1e-10,
    max_step=30.0
)

for result in (hcw, target, chaser):
    if not result.success:
        raise RuntimeError(result.message)


# 4. Convert nonlinear ECI trajectories into LVLH
nonlinear_lvlh = np.zeros_like(hcw.y)

for i in range(len(times)):
    rho, rho_dot = eci_to_lvlh(
        target.y[:3, i],
        target.y[3:, i],
        chaser.y[:3, i],
        chaser.y[3:, i],
        n
    )

    nonlinear_lvlh[:3, i] = rho
    nonlinear_lvlh[3:, i] = rho_dot


# 5. Compare the two models
position_difference = (
    nonlinear_lvlh[:3] - hcw.y[:3]
)

velocity_difference = (
    nonlinear_lvlh[3:] - hcw.y[3:]
)

position_errors = np.linalg.norm(
    position_difference, axis=0
)

velocity_errors = np.linalg.norm(
    velocity_difference, axis=0
)


print("--- HCW VS NONLINEAR TWO-BODY ---")

print("\nFinal HCW position (m):")
print(hcw.y[:3, -1])

print("\nFinal nonlinear position in LVLH (m):")
print(nonlinear_lvlh[:3, -1])

print("\nFinal position difference (m):")
print(position_difference[:, -1])

print("\n--- MODEL AGREEMENT ---")
print(f"Final position error: {position_errors[-1]:.6f} m")
print(f"Maximum position error: {position_errors.max():.6f} m")
print(f"Final velocity error: {velocity_errors[-1]:.9f} m/s")
print(f"Maximum velocity error: {velocity_errors.max():.9f} m/s")
