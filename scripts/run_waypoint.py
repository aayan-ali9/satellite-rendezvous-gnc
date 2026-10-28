
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.constants import circular_mean_motion
from rendezvous.lqr import hcw_state_space, lqr_gain
from rendezvous.guidance import waypoint_reached


# 1. Set up reference orbit and dynamics
n = circular_mean_motion(500_000)
A, B = hcw_state_space(n)

Q = np.diag([1e-6, 1e-6, 1e-6,
             1e-2, 1e-2, 1e-2])
R = np.eye(3) * 1e6

K = lqr_gain(A, B, Q, R)


# 2. Define initial state and first waypoint
initial_state = np.array([
    100.0, -995.0, 0.0,
    0.0, 0.0, 0.0
])

waypoint = np.array([
    0.0, -200.0, 0.0,
    0.0, 0.0, 0.0
])


# 3. LQR controller and HCW dynamics
def controlled_derivative(t, state):
    error = state - waypoint
    u = -K @ error

    return A @ state + B @ u


# 4. Simulate four orbital periods
period_s = 2 * np.pi / n
times = np.linspace(0, 4 * period_s, 1201)

solution = solve_ivp(
    controlled_derivative,
    (times[0], times[-1]),
    initial_state,
    t_eval=times,
    rtol=1e-9,
    atol=1e-11
)

if not solution.success:
    raise RuntimeError(solution.message)


# 5. Check when waypoint conditions are met
accepted_indices = []

for i, state in enumerate(solution.y.T):
    if waypoint_reached(state, waypoint):
        accepted_indices.append(i)


print("--- WAYPOINT GUIDANCE TEST ---")

if accepted_indices:
    i = accepted_indices[0]

    print("Waypoint reached: YES")
    print(f"First sampled acceptance time: {solution.t[i]:.1f} s")

    position_error = np.linalg.norm(
        solution.y[:3, i] - waypoint[:3]
    )
    velocity_error = np.linalg.norm(
        solution.y[3:, i] - waypoint[3:]
    )

    print(f"Position error: {position_error:.4f} m")
    print(f"Velocity error: {velocity_error:.6f} m/s")

else:
    print("Waypoint reached: NO")


# 6. Monitor target separation
ranges = np.linalg.norm(solution.y[:3, :], axis=0)

print(f"Minimum sampled target separation: {ranges.min():.3f} m")
print(f"Final target separation: {ranges[-1]:.3f} m")
