
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.twobody import (
    MU_EARTH,
    two_body_derivative
)

# Initial circular orbit
radius = 6_878_000.0

speed = np.sqrt(MU_EARTH / radius)

initial_state = np.array([
    radius, 0.0, 0.0,
    0.0, speed, 0.0
])

# Calculate orbital period
n = np.sqrt(MU_EARTH / radius**3)
period = 2 * np.pi / n

# Propagate one complete orbit
solution = solve_ivp(
    two_body_derivative,
    (0.0, period),
    initial_state,
    t_eval=np.linspace(0.0, period, 500),
    rtol=1e-10,
    atol=1e-10,
    max_step=30.0
)

if not solution.success:
    raise RuntimeError(solution.message)

# Extract final state
final_state = solution.y[:, -1]

# Calculate position and velocity errors
position_error = np.linalg.norm(
    final_state[:3] - initial_state[:3]
)

velocity_error = np.linalg.norm(
    final_state[3:] - initial_state[3:]
)

# Check how much the orbital radius changes
radii = np.linalg.norm(solution.y[:3], axis=0)

print(f"Orbital period: {period:.3f} s")
print(f"Initial radius: {radius:.3f} m")
print(f"Minimum radius: {radii.min():.3f} m")
print(f"Maximum radius: {radii.max():.3f} m")

print("\n--- ORBITAL RETURN CHECK ---")
print(f"Position error: {position_error:.9f} m")
print(f"Velocity error: {velocity_error:.12f} m/s")
