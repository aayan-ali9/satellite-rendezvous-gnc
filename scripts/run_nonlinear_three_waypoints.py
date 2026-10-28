
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.integrate import solve_ivp, trapezoid

from rendezvous.twobody import MU_EARTH
from rendezvous.frames import lvlh_to_eci, eci_to_lvlh
from rendezvous.lqr import hcw_state_space, lqr_gain
from rendezvous.nonlinear_control import nonlinear_controlled_derivative
from rendezvous.constants import circular_mean_motion
from rendezvous.guidance import waypoint_acceptance_margin



# 1. Reference orbit and controller

# Match the mean motion used in our HCW baseline
n = circular_mean_motion(500_000)

# Choose a circular orbit consistent with that mean motion
radius = (MU_EARTH / n**2)**(1.0 / 3.0)

speed = np.sqrt(MU_EARTH / radius)


period = 2 * np.pi / n
end_time = 4 * period

A, B = hcw_state_space(n)

Q = np.diag([
    1e-6, 1e-6, 1e-6,
    1e-2, 1e-2, 1e-2
])

R = np.eye(3) * 1e6
K = lqr_gain(A, B, Q, R)


# 2. Guidance settings
POSITION_TOLERANCE = 5.0
VELOCITY_TOLERANCE = 0.10

SCREENING_RANGE = 25.0
MAX_ACCELERATION = 0.001

waypoints = [
    np.array([0, -200, 0, 0, 0, 0], dtype=float),
    np.array([0, -50, 0, 0, 0, 0], dtype=float),
    np.array([0, -30, 0, 0, 0, 0], dtype=float)
]


# 3. Initialize target and chaser in ECI
r_target = np.array([radius, 0.0, 0.0])
v_target = np.array([0.0, speed, 0.0])

target_initial = np.concatenate((r_target, v_target))


# Nominal initial relative state
baseline_initial = np.array([
    100.0, -995.0, 0.0,
    0.0, 0.0, 0.0
])

# Deliberate initial-condition perturbation

initial_perturbation = np.array([
    0.0, 0.0, 0.0,
    0.01, -0.01, 0.005
])


relative_initial = baseline_initial + initial_perturbation

print("Initial relative state:", relative_initial)


r_chaser, v_chaser = lvlh_to_eci(
    r_target,
    v_target,
    relative_initial[:3],
    relative_initial[3:],
    n
)

chaser_initial = np.concatenate((r_chaser, v_chaser))

current_state = np.concatenate((
    target_initial,
    chaser_initial
))

current_time = 0.0
total_delta_v = 0.0
stage_trajectories = []
stage_times = []

# 4. Convert combined ECI state to LVLH
def relative_lvlh(state):

    target = state[:6]
    chaser = state[6:]

    rho, rho_dot = eci_to_lvlh(
        target[:3],
        target[3:],
        chaser[:3],
        chaser[3:],
        n
    )

    return np.concatenate((rho, rho_dot))


# 5. Simulated separation boundary event
def entered_buffer(t, state):

    separation = np.linalg.norm(
        state[6:9] - state[:3]
    )

    return separation - SCREENING_RANGE


entered_buffer.terminal = True
entered_buffer.direction = -1


# 6. Execute nonlinear waypoint sequence
for stage_number, waypoint in enumerate(waypoints, start=1):

    stop_at_waypoint = stage_number < 3

    
    def waypoint_accepted(t, state):
        relative = relative_lvlh(state)

        return waypoint_acceptance_margin(
            relative,
            waypoint,
            POSITION_TOLERANCE,
            VELOCITY_TOLERANCE
        )

    waypoint_accepted.terminal = True
    waypoint_accepted.direction = -1

    events = (
        [waypoint_accepted, entered_buffer]
        if stop_at_waypoint
        else [entered_buffer]
    )

    solution = solve_ivp(
        lambda t, state: nonlinear_controlled_derivative(
            t, state, waypoint, n, K, MAX_ACCELERATION
        ),
        (current_time, end_time),
        current_state,
        events=events,
        rtol=1e-10,
        atol=1e-10,
        max_step=2.5
    )

    if not solution.success:
        raise RuntimeError(solution.message)

    # Reject a simulated screening-boundary crossing
    if len(solution.t_events[-1]) > 0:
        raise RuntimeError(
            f"Stage {stage_number}: "
            "25 m screening boundary crossed."
        )

    # Intermediate stages must satisfy acceptance
    if stop_at_waypoint:
        if len(solution.t_events[0]) == 0:
            raise RuntimeError(
                f"Stage {stage_number}: "
                "Waypoint was not accepted."
            )

    # 7. Calculate relative states and control effort
    relative_history = np.array([
        relative_lvlh(solution.y[:, i])
        for i in range(solution.t.size)
    ])
    stage_trajectories.append(relative_history.copy())
    stage_times.append(solution.t.copy())
    errors = relative_history - waypoint

    requested = -(errors @ K.T)

    requested_magnitudes = np.linalg.norm(
        requested, axis=1
    )

    actual_magnitudes = np.minimum(
        requested_magnitudes,
        MAX_ACCELERATION
    )

    stage_delta_v = trapezoid(
        actual_magnitudes,
        x=solution.t
    )

    total_delta_v += stage_delta_v

    separations = np.linalg.norm(
        solution.y[6:9] - solution.y[:3],
        axis=0
    )

    final_relative = relative_history[-1]

    stage_position_error = np.linalg.norm(
        final_relative[:3] - waypoint[:3]
    )

    stage_velocity_error = np.linalg.norm(
        final_relative[3:] - waypoint[3:]
    )

    # 8. Print stage diagnostics
    print(f"\n--- NONLINEAR STAGE {stage_number} ---")
    print(f"Waypoint: {-waypoint[1]:.1f} m")
    print(f"Stage end time: {solution.t[-1]:.1f} s")
    print(f"Minimum separation: {separations.min():.3f} m")
    print(f"Position error: {stage_position_error:.6f} m")
    print(f"Velocity error: {stage_velocity_error:.9f} m/s")
    print(f"Peak actual acceleration: "
          f"{actual_magnitudes.max():.7f} m/s^2")
    print(f"Stage delta-v: {stage_delta_v:.6f} m/s")

    # Transition using the nonlinear simulation state
    current_time = solution.t[-1]
    current_state = solution.y[:, -1].copy()


# 9. Final mission evaluation
final_relative = relative_lvlh(current_state)
final_waypoint = waypoints[-1]

position_error = np.linalg.norm(
    final_relative[:3] - final_waypoint[:3]
)

velocity_error = np.linalg.norm(
    final_relative[3:] - final_waypoint[3:]
)

print("\n--- NONLINEAR MISSION SUMMARY ---")
print(f"Total mission delta-v: {total_delta_v:.6f} m/s")
print(f"Final position error: {position_error:.6f} m")
print(f"Final velocity error: {velocity_error:.9f} m/s")

if (position_error <= POSITION_TOLERANCE
        and velocity_error <= VELOCITY_TOLERANCE):
    print("Final waypoint acceptance: PASS")
else:
    print("Final waypoint acceptance: FAIL")

# 10. Visualize the nonlinear rendezvous trajectory

fig, ax = plt.subplots(figsize=(10, 6))

for i, trajectory in enumerate(stage_trajectories, start=1):

    # LVLH along-track position is y (column 1)
    # LVLH radial position is x (column 0)
    ax.plot(
        trajectory[:, 1],
        trajectory[:, 0],
        label=f"Stage {i}",
        linewidth=2
    )

# Mark desired waypoints
for i, waypoint in enumerate(waypoints, start=1):
    ax.scatter(
        waypoint[1],
        waypoint[0],
        marker="x",
        s=90
    )
    ax.annotate(
        f"WP{i}",
        (waypoint[1], waypoint[0]),
        xytext=(6, 8),
        textcoords="offset points"
    )

# Mark initial position and target
ax.scatter(
    relative_initial[1],
    relative_initial[0],
    marker="o",
    s=80,
    label="Start"
)

ax.scatter(
    0, 0,
    marker="*",
    s=140,
    label="Target"
)

ax.set_title("Nonlinear Three-Waypoint Rendezvous")
ax.set_xlabel("Along-track y (m)")
ax.set_ylabel("Radial x (m)")
ax.set_aspect("equal", adjustable="box")
ax.grid(True)
ax.legend()

fig.tight_layout()

# Save figure for the portfolio
output_dir = Path("figures")
output_dir.mkdir(exist_ok=True)

fig.savefig(
    output_dir / "nonlinear_trajectory.png",
    dpi=300
)

# 11. Close-up of final approach and safety boundaries

fig2, ax2 = plt.subplots(figsize=(9, 7))

# Plot stages 2 and 3
ax2.plot(
    stage_trajectories[1][:, 1],
    stage_trajectories[1][:, 0],
    color="tab:orange",
    linewidth=2,
    label="Stage 2"
)

ax2.plot(
    stage_trajectories[2][:, 1],
    stage_trajectories[2][:, 0],
    color="tab:green",
    linewidth=2,
    label="Stage 3"
)

# Hard keep-out boundary: 20 m
keepout = plt.Circle(
    (0, 0),
    20,
    fill=False,
    color="red",
    linestyle="--",
    linewidth=2,
    label="20 m keep-out"
)

# Screening boundary: 25 m
screening = plt.Circle(
    (0, 0),
    25,
    fill=False,
    color="orange",
    linestyle=":",
    linewidth=2,
    label="25 m screening"
)

ax2.add_patch(keepout)
ax2.add_patch(screening)

# Mark the target and the two close-range waypoints
ax2.scatter(
    0, 0,
    marker="*",
    s=180,
    color="purple",
    label="Target"
)

ax2.scatter(
    [-50, -30],
    [0, 0],
    marker="x",
    s=100,
    color="black",
    label="Waypoints"
)

ax2.annotate("WP2", (-50, 0), xytext=(-5, 15),
             textcoords="offset points")

ax2.annotate("WP3", (-30, 0), xytext=(5, 15),
             textcoords="offset points")

ax2.set_xlim(-95, 10)
ax2.set_ylim(-55, 55)

ax2.set_aspect("equal", adjustable="box")
ax2.set_xlabel("Along-track y (m)")
ax2.set_ylabel("Radial x (m)")
ax2.set_title("Final Rendezvous Approach and Safety Boundaries")

ax2.grid(True)
ax2.legend(loc="upper left")

fig2.tight_layout()

fig2.savefig(
    output_dir / "final_approach_safety.png",
    dpi=300
)


# 12. Final-stage separation versus time

final_times = stage_times[2]
final_trajectory = stage_trajectories[2]

# Calculate actual 3D relative distance
final_ranges = np.linalg.norm(
    final_trajectory[:, :3],
    axis=1
)

# Time since beginning of Stage 3, in hours
elapsed_hours = (
    final_times - final_times[0]
) / 3600.0

fig3, ax3 = plt.subplots(figsize=(10, 6))

ax3.plot(
    elapsed_hours,
    final_ranges,
    linewidth=2,
    label="Nonlinear separation"
)

ax3.axhline(
    30,
    color="green",
    linestyle="--",
    label="30 m desired hold"
)

ax3.axhline(
    25,
    color="orange",
    linestyle=":",
    label="25 m screening"
)

ax3.axhline(
    20,
    color="red",
    linestyle="--",
    label="20 m keep-out"
)

ax3.set_xlabel("Time since Stage 3 began (hours)")
ax3.set_ylabel("3D separation (m)")
ax3.set_title("Final Holding Point: Separation vs. Time")

ax3.set_ylim(18, 55)
ax3.grid(True)
ax3.legend()

fig3.tight_layout()

fig3.savefig(
    output_dir / "final_holding_separation.png",
    dpi=300
)


plt.show()
