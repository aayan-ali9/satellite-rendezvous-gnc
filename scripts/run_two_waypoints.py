
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.constants import circular_mean_motion
from rendezvous.lqr import hcw_state_space, lqr_gain

# Orbital dynamics and controller
n = circular_mean_motion(500_000)
A, B = hcw_state_space(n)

Q = np.diag([1e-6, 1e-6, 1e-6,
             1e-2, 1e-2, 1e-2])
R = np.eye(3) * 1e6

K = lqr_gain(A, B, Q, R)

initial_state = np.array(
    [100, -995, 0, 0, 0, 0], dtype=float
)

waypoint_200 = np.array(
    [0, -200, 0, 0, 0, 0], dtype=float
)

waypoint_50 = np.array(
    [0, -50, 0, 0, 0, 0], dtype=float
)

position_tolerance = 5.0
velocity_tolerance = 0.10


# Controlled HCW dynamics for a selected waypoint
def make_derivative(waypoint):
    def derivative(t, state):
        error = state - waypoint
        u = -K @ error

        return A @ state + B @ u

    return derivative


# Continuous acceptance boundary for 200 m waypoint
def waypoint_event(t, state):
    position_error = np.linalg.norm(
        state[:3] - waypoint_200[:3]
    )
    velocity_error = np.linalg.norm(
        state[3:] - waypoint_200[3:]
    )

    return max(
        position_error / position_tolerance,
        velocity_error / velocity_tolerance
    ) - 1.0


# Stop first integration when both tolerances are met
waypoint_event.terminal = True
waypoint_event.direction = -1

period_s = 2 * np.pi / n
end_time = 4 * period_s


# Stage 1: Approach the 200 m waypoint
stage1 = solve_ivp(
    make_derivative(waypoint_200),
    (0, end_time),
    initial_state,
    events=waypoint_event,
    rtol=1e-9,
    atol=1e-11,
    max_step=15.0
)

if not stage1.success:
    raise RuntimeError(stage1.message)

if len(stage1.t_events[0]) == 0:
    raise RuntimeError(
        "200 m waypoint was not accepted."
    )

# Exact detected event time and state
switch_time = stage1.t_events[0][0]
switch_state = stage1.y_events[0][0]

print("--- GUIDANCE SWITCH ---")
print(f"200 m waypoint accepted at: {switch_time:.1f} s")
print("Switching target to 50 m")


# Stage 2: Approach and hold the 50 m waypoint

# Stage 2: Predict the proposed 50 m approach
candidate = solve_ivp(
    make_derivative(waypoint_50),
    (switch_time, end_time),
    switch_state,
    rtol=1e-9,
    atol=1e-11,
    max_step=5.0
)

if not candidate.success:
    raise RuntimeError(candidate.message)

# Check the predicted trajectory against the target
predicted_ranges = np.linalg.norm(
    candidate.y[:3], axis=0
)

predicted_min_range = predicted_ranges.min()

MIN_SEPARATION_M = 20.0
SAFETY_MARGIN_M = 5.0

required_range = MIN_SEPARATION_M + SAFETY_MARGIN_M

print("\n--- PREDICTIVE SAFETY CHECK ---")
print(f"Predicted minimum range: {predicted_min_range:.3f} m")
print(f"Required range with margin: {required_range:.3f} m")

if predicted_min_range < required_range:
    print("Approach authorization: DENIED")
    raise RuntimeError(
        "Proposed approach violates safety margin."
    )

print("Approach authorization: APPROVED")

# Use the approved ideal-model prediction as Stage 2
stage2 = candidate

# Evaluate final state
final_state = stage2.y[:, -1]

position_error = np.linalg.norm(
    final_state[:3] - waypoint_50[:3]
)
velocity_error = np.linalg.norm(
    final_state[3:] - waypoint_50[3:]
)

# Minimum sampled target separation across both stages
ranges1 = np.linalg.norm(stage1.y[:3], axis=0)
ranges2 = np.linalg.norm(stage2.y[:3], axis=0)
minimum_range = min(ranges1.min(), ranges2.min())

print("\n--- FINAL RESULTS ---")
print(f"Position error: {position_error:.6f} m")
print(f"Velocity error: {velocity_error:.9f} m/s")
print(f"Minimum sampled target range: {minimum_range:.3f} m")

if minimum_range < 20.0:
    print("20 m sampled safety check: FAIL")
else:
    print("20 m sampled safety check: PASS")

import matplotlib.pyplot as plt
from pathlib import Path

results_dir = Path("results")
results_dir.mkdir(exist_ok=True)

fig, ax = plt.subplots(figsize=(9, 6))

# Stage 1: initial position to 200 m waypoint
ax.plot(
    stage1.y[1],
    stage1.y[0],
    label="Stage 1: Approach 200 m",
    linewidth=2
)

# Stage 2: 200 m waypoint to 50 m waypoint
ax.plot(
    stage2.y[1],
    stage2.y[0],
    label="Stage 2: Approach 50 m",
    linewidth=2
)

# Important points
ax.scatter(
    initial_state[1], initial_state[0],
    marker="o", s=60, label="Start"
)

ax.scatter(
    switch_state[1], switch_state[0],
    marker="s", s=70, label="Guidance switch"
)

ax.scatter(
    waypoint_50[1], waypoint_50[0],
    marker="x", s=100, label="50 m waypoint"
)

ax.scatter(
    0, 0, marker="*", s=150, label="Target"
)

# 20 m boundary in the x-y plane.
# Valid for this simulation because z remains zero.
safety_circle = plt.Circle(
    (0, 0), 20,
    fill=False, linestyle="--",
    color="red", label="20 m boundary"
)
ax.add_patch(safety_circle)

ax.set_xlabel("Along-track y (m)")
ax.set_ylabel("Radial x (m)")
ax.set_title("Two-Waypoint LQR Rendezvous")
ax.grid(True)
ax.legend()
ax.set_aspect("equal", adjustable="box")

plt.tight_layout()
fig.savefig(
    results_dir / "two_waypoint_trajectory.png",
    dpi=180
)

# Create a second plot zoomed near the target
ax.set_xlim(-230, 30)
ax.set_ylim(-140, 140)

fig.savefig(
    results_dir / "two_waypoint_closeup.png",
    dpi=180
)

plt.close(fig)

print("Trajectory plots saved.")
