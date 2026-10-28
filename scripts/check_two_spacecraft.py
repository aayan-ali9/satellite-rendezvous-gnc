
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.twobody import MU_EARTH, two_body_derivative
from rendezvous.frames import lvlh_to_eci


# 1. Initialize the target in a circular orbit
radius = 6_878_000.0

speed = np.sqrt(MU_EARTH / radius)
n = np.sqrt(MU_EARTH / radius**3)
period = 2 * np.pi / n

r_target = np.array([radius, 0.0, 0.0])
v_target = np.array([0.0, speed, 0.0])

target_initial = np.concatenate((r_target, v_target))


# 2. Initialize the chaser relative to the target
rho = np.array([100.0, 0.0, 0.0])
rho_dot = np.array([0.0, 0.0, 0.0])

r_chaser, v_chaser = lvlh_to_eci(
    r_target, v_target, rho, rho_dot, n
)

chaser_initial = np.concatenate((r_chaser, v_chaser))


# 3. Propagate both spacecraft independently
times = np.linspace(0.0, period, 500)

target_solution = solve_ivp(
    two_body_derivative,
    (0.0, period),
    target_initial,
    t_eval=times,
    rtol=1e-10,
    atol=1e-10,
    max_step=30.0
)

chaser_solution = solve_ivp(
    two_body_derivative,
    (0.0, period),
    chaser_initial,
    t_eval=times,
    rtol=1e-10,
    atol=1e-10,
    max_step=30.0
)

if not target_solution.success:
    raise RuntimeError(target_solution.message)

if not chaser_solution.success:
    raise RuntimeError(chaser_solution.message)


# 4. Calculate physical separation over time
relative_position_eci = (
    chaser_solution.y[:3] - target_solution.y[:3]
)

separations = np.linalg.norm(relative_position_eci, axis=0)


print("--- TWO-SPACECRAFT PROPAGATION ---")
print(f"Initial separation: {separations[0]:.6f} m")
print(f"Final separation: {separations[-1]:.6f} m")
print(f"Minimum separation: {separations.min():.6f} m")
print(f"Maximum separation: {separations.max():.6f} m")
print(f"Simulation duration: {period:.3f} s")
