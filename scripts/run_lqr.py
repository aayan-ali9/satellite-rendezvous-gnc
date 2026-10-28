
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.integrate import solve_ivp

from rendezvous.constants import circular_mean_motion
from rendezvous.lqr import hcw_state_space, lqr_gain


# 1. Reference orbit
altitude_m = 500_000.0
n = circular_mean_motion(altitude_m)
period_s = 2 * np.pi / n

# 2. Initial and desired relative states
# Order: [x, y, z, vx, vy, vz]
initial_state = np.array([
    100.0, -995.0, 0.0,
    0.0, 0.0, 0.0
])

desired_state = np.array([
    0.0, -20.0, 0.0,
    0.0, 0.0, 0.0
])

# 3. Build HCW matrices and calculate LQR gain
A, B = hcw_state_space(n)

Q = np.diag([
    1e-6, 1e-6, 1e-6,
    1e-2, 1e-2, 1e-2
])
R = np.eye(3) * 1e6

K = lqr_gain(A, B, Q, R)

# Check the holding point is an HCW equilibrium
assert np.allclose(A @ desired_state, 0.0)


# 4. Controlled state derivative
def controlled_derivative(t, state):
    error = state - desired_state
    u = -K @ error

    return A @ state + B @ u


# 5. Numerically propagate for four orbital periods
times_s = np.linspace(0, 4 * period_s, 1201)

solution = solve_ivp(
    controlled_derivative,
    (times_s[0], times_s[-1]),
    initial_state,
    t_eval=times_s,
    rtol=1e-9,
    atol=1e-11
)

if not solution.success:
    raise RuntimeError(solution.message)


# 6. Evaluate performance
errors = solution.y - desired_state[:, None]
commands = -K @ errors

position_errors = np.linalg.norm(errors[:3], axis=0)
velocity_errors = np.linalg.norm(errors[3:], axis=0)
accel_magnitudes = np.linalg.norm(commands, axis=0)

eigenvalues = np.linalg.eigvals(A - B @ K)

print("Closed-loop stable:",
      np.all(np.real(eigenvalues) < 0))
print(f"Initial position error: {position_errors[0]:.3f} m")
print(f"Final position error:   {position_errors[-1]:.6f} m")
print(f"Final velocity error:   {velocity_errors[-1]:.9f} m/s")
print(f"Maximum commanded acceleration: "
      f"{accel_magnitudes.max():.6f} m/s^2")


# 7. Save plots
results_dir = Path("results")
results_dir.mkdir(exist_ok=True)

# Relative trajectory
plt.figure(figsize=(8, 6))
plt.plot(solution.y[1], solution.y[0], label="LQR trajectory")
plt.scatter(initial_state[1], initial_state[0],
            label="Start", marker="o")
plt.scatter(desired_state[1], desired_state[0],
            label="Holding point", marker="x", s=100)
plt.xlabel("Along-track y (m)")
plt.ylabel("Radial x (m)")
plt.title("Controlled HCW Relative Trajectory")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.tight_layout()
plt.savefig(results_dir / "lqr_trajectory.png", dpi=180)
plt.close()

# Position error over time
plt.figure(figsize=(8, 5))
plt.plot(solution.t / 3600, position_errors)
plt.xlabel("Time (hours)")
plt.ylabel("Position error (m)")
plt.title("LQR Position Error Convergence")
plt.grid(True)
plt.tight_layout()
plt.savefig(results_dir / "lqr_position_error.png", dpi=180)
plt.close()

# Commanded acceleration magnitude
plt.figure(figsize=(8, 5))
plt.plot(solution.t / 3600, accel_magnitudes)
plt.xlabel("Time (hours)")
plt.ylabel("Commanded acceleration (m/s^2)")
plt.title("LQR Control Effort")
plt.grid(True)
plt.tight_layout()
plt.savefig(results_dir / "lqr_control_effort.png", dpi=180)
plt.close()

print("Plots saved in results/")

from scipy.integrate import trapezoid

# Distance from the actual target at [0, 0, 0]
target_ranges = np.linalg.norm(solution.y[:3], axis=0)

# Closest approach to the target
print(f"Minimum target separation: {target_ranges.min():.3f} m")

# Ideal cumulative delta-v from continuous commanded acceleration
delta_v = trapezoid(accel_magnitudes, x=solution.t)
print(f"Total commanded delta-v: {delta_v:.4f} m/s")

# 8. Mission safety requirement check

MIN_SEPARATION_M = 20.0
TOLERANCE_M = 1e-6

# Target is located at the LVLH origin
ranges = np.linalg.norm(solution.y[:3, :], axis=0)

# Find closest sampled approach
closest_index = np.argmin(ranges)
minimum_range = ranges[closest_index]
closest_time = solution.t[closest_index]

# Check the minimum separation requirement
violation = ranges < (MIN_SEPARATION_M - TOLERANCE_M)

print("\n--- MISSION SAFETY CHECK ---")
print(f"Required minimum separation: {MIN_SEPARATION_M:.3f} m")
print(f"Minimum sampled separation:  {minimum_range:.3f} m")
print(f"Closest sampled time:        {closest_time:.1f} s")

if np.any(violation):
    first_violation = np.flatnonzero(violation)[0]
    print("SAFETY RESULT: FAIL")
    print(f"First sampled violation: "
          f"{solution.t[first_violation]:.1f} s")
else:
    print("SAFETY RESULT: PASS (sampled points only)")
