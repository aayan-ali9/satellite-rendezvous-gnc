
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.twobody import MU_EARTH
from rendezvous.frames import lvlh_to_eci, eci_to_lvlh
from rendezvous.lqr import hcw_state_space, lqr_gain
from rendezvous.control import saturate_acceleration
from rendezvous.nonlinear_control import nonlinear_controlled_derivative


# 1. Target circular orbit
radius = 6_878_000.0
speed = np.sqrt(MU_EARTH / radius)
n = np.sqrt(MU_EARTH / radius**3)

period = 2 * np.pi / n
duration = 1.5 * period

r_target = np.array([radius, 0.0, 0.0])
v_target = np.array([0.0, speed, 0.0])

target_initial = np.concatenate((r_target, v_target))


# 2. Reuse our existing LQR design
A, B = hcw_state_space(n)

Q = np.diag([
    1e-6, 1e-6, 1e-6,
    1e-2, 1e-2, 1e-2
])

R = np.eye(3) * 1e6

K = lqr_gain(A, B, Q, R)

MAX_ACCELERATION = 0.001


# 3. Initial relative state and waypoint
relative_initial = np.array([
    100.0, -995.0, 0.0,
    0.0, 0.0, 0.0
])

waypoint = np.array([
    0.0, -200.0, 0.0,
    0.0, 0.0, 0.0
])


# 4. Convert initial LVLH state into ECI
r_chaser, v_chaser = lvlh_to_eci(
    r_target,
    v_target,
    relative_initial[:3],
    relative_initial[3:],
    n
)

chaser_initial = np.concatenate((r_chaser, v_chaser))

nonlinear_initial = np.concatenate((
    target_initial,
    chaser_initial
))


# 5. Define controlled HCW dynamics
def hcw_controlled(t, state):
    error = state - waypoint

    requested = -K @ error

    actual = saturate_acceleration(
        requested,
        MAX_ACCELERATION
    )

    return A @ state + B @ actual


# 6. Propagate both models
times = np.linspace(0.0, duration, 800)

hcw = solve_ivp(
    hcw_controlled,
    (0.0, duration),
    relative_initial,
    t_eval=times,
        rtol=1e-11,
        atol=1e-12,
        max_step=2.5

)

nonlinear = solve_ivp(
    lambda t, state: nonlinear_controlled_derivative(
        t, state, waypoint, n, K, MAX_ACCELERATION
    ),
    (0.0, duration),
    nonlinear_initial,
    t_eval=times,
    rtol=1e-10,
    atol=1e-10,
    max_step=5.0
)

if not hcw.success:
    raise RuntimeError(hcw.message)

if not nonlinear.success:
    raise RuntimeError(nonlinear.message)


# 7. Convert nonlinear trajectory back into LVLH
nonlinear_lvlh = np.zeros_like(hcw.y)

for i in range(len(times)):

    target_state = nonlinear.y[:6, i]
    chaser_state = nonlinear.y[6:, i]

    rho, rho_dot = eci_to_lvlh(
        target_state[:3],
        target_state[3:],
        chaser_state[:3],
        chaser_state[3:],
        n
    )

    nonlinear_lvlh[:, i] = np.concatenate((
        rho, rho_dot
    ))


# 8. Compare trajectories
position_differences = np.linalg.norm(
    nonlinear_lvlh[:3] - hcw.y[:3],
    axis=0
)

velocity_differences = np.linalg.norm(
    nonlinear_lvlh[3:] - hcw.y[3:],
    axis=0
)

hcw_final_error = np.linalg.norm(
    hcw.y[:3, -1] - waypoint[:3]
)

nonlinear_final_error = np.linalg.norm(
    nonlinear_lvlh[:3, -1] - waypoint[:3]
)

nonlinear_velocity_error = np.linalg.norm(
    nonlinear_lvlh[3:, -1] - waypoint[3:]
)

nonlinear_ranges = np.linalg.norm(
    nonlinear_lvlh[:3],
    axis=0
)


print("--- CONTROLLED MODEL COMPARISON ---")
print(f"Simulation duration: {duration:.1f} s")

print("\n--- FINAL WAYPOINT PERFORMANCE ---")
print(f"HCW position error: {hcw_final_error:.6f} m")
print(f"Nonlinear position error: {nonlinear_final_error:.6f} m")
print(f"Nonlinear velocity error: {nonlinear_velocity_error:.9f} m/s")

print("\n--- MODEL DISAGREEMENT ---")
print(f"Final position difference: {position_differences[-1]:.6f} m")
print(f"Maximum position difference: {position_differences.max():.6f} m")
print(f"Maximum velocity difference: {velocity_differences.max():.9f} m/s")

print("\n--- SEPARATION CHECK ---")
print(f"Minimum nonlinear separation: {nonlinear_ranges.min():.3f} m")
