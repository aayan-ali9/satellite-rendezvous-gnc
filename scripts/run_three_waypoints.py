
from rendezvous.control import saturate_acceleration
import numpy as np
from scipy.integrate import solve_ivp, trapezoid

from rendezvous.constants import circular_mean_motion
from rendezvous.lqr import hcw_state_space, lqr_gain


# 1. Orbital dynamics and LQR controller
n = circular_mean_motion(500_000)
period_s = 2 * np.pi / n
end_time = 4 * period_s

A, B = hcw_state_space(n)

Q = np.diag([1e-6, 1e-6, 1e-6,
             1e-2, 1e-2, 1e-2])
R = np.eye(3) * 1e6

K = lqr_gain(A, B, Q, R)


# 2. Initial state and approach waypoints
initial_state = np.array(
    [100, -995, 0, 0, 0, 0], dtype=float
)

waypoints = [
    np.array([0, -200, 0, 0, 0, 0], dtype=float),
    np.array([0, -50, 0, 0, 0, 0], dtype=float),
    np.array([0, -30, 0, 0, 0, 0], dtype=float),
]

POSITION_TOLERANCE = 5.0
VELOCITY_TOLERANCE = 0.10
SCREENING_RANGE = 25.0
MAX_ACCELERATION = 0.001  # m/s^2



# 3. Predict motion toward a selected waypoint
def predict_stage(start_time, start_state,
                  waypoint, stop_at_waypoint):

    def dynamics(t, state):
        error = state - waypoint

        # LQR requests an acceleration
        requested_control = -K @ error

        # Thruster limits determine actual acceleration
        actual_control = saturate_acceleration(
            requested_control,
            MAX_ACCELERATION
        )

        # Propagate using available acceleration
        return A @ state + B @ actual_control


    # Waypoint acceptance condition
    def accepted(t, state):
        position_error = np.linalg.norm(
            state[:3] - waypoint[:3]
        )
        velocity_error = np.linalg.norm(
            state[3:] - waypoint[3:]
        )

        return max(
            position_error / POSITION_TOLERANCE,
            velocity_error / VELOCITY_TOLERANCE
        ) - 1.0

    accepted.terminal = True
    accepted.direction = -1

    # Detect entry into the 25 m screening boundary
    def entered_buffer(t, state):
        return np.linalg.norm(state[:3]) - SCREENING_RANGE

    entered_buffer.terminal = True
    entered_buffer.direction = -1

    if stop_at_waypoint:
        events = [accepted, entered_buffer]
    else:
        events = [entered_buffer]

    candidate = solve_ivp(
        dynamics,
        (start_time, end_time),
        start_state,
        events=events,
        rtol=1e-9,
        atol=1e-11,
        max_step=5.0
    )

    if not candidate.success:
        raise RuntimeError(candidate.message)

    # Reject a predicted boundary crossing
    if len(candidate.t_events[-1]) > 0:
        raise RuntimeError(
            "DENIED: Predicted trajectory reaches "
            "the 25 m screening boundary."
        )

    # Intermediate waypoints must be accepted
    if stop_at_waypoint:
        if len(candidate.t_events[0]) == 0:
            raise RuntimeError(
                "Waypoint not reached within simulation time."
            )

    return candidate


# 4. Execute ideal-model waypoint sequence
current_time = 0.0
current_state = initial_state.copy()
total_delta_v = 0.0

for stage_number, waypoint in enumerate(waypoints, start=1):

    # Stage 3 is the final hold: simulate to end_time
    stop_at_waypoint = (stage_number < 3)

    candidate = predict_stage(
        current_time,
        current_state,
        waypoint,
        stop_at_waypoint
    )


    ranges = np.linalg.norm(candidate.y[:3], axis=0)

    errors = candidate.y.T - waypoint
    requested = -(errors @ K.T)

    requested_magnitudes = np.linalg.norm(
        requested, axis=1
    )

    actual_magnitudes = np.minimum(
        requested_magnitudes, MAX_ACCELERATION
    
    )
    
    # Integrate the actual acceleration magnitude over time
    stage_delta_v = trapezoid(
        actual_magnitudes,
        x=candidate.t
    )

    total_delta_v += stage_delta_v





    saturated = requested_magnitudes > MAX_ACCELERATION + 1e-12

    print(f"\n--- STAGE {stage_number} ---")
    print("Predictive screening: APPROVED")
    print(f"Desired holding range: {-waypoint[1]:.1f} m")
    print(f"Minimum sampled range: {ranges.min():.3f} m")
    print(f"Stage end time: {candidate.t[-1]:.1f} s")
    print(f"Peak requested acceleration: "
          f"{requested_magnitudes.max():.7f} m/s^2")
    print(f"Peak actual acceleration: "
          f"{actual_magnitudes.max():.7f} m/s^2")
    
    print(f"Saturated solver samples: "
          f"{np.count_nonzero(saturated)} / {len(saturated)}")

    print(f"Stage delta-v: {stage_delta_v:.6f} m/s")

    # Preserve the actual modeled state at transition
    current_time = candidate.t[-1]
    current_state = candidate.y[:, -1].copy()


# Print total delta-v after all stages are complete
print(f"\nTotal mission delta-v: {total_delta_v:.6f} m/s")

# 5. Evaluate final holding performance
final_waypoint = waypoints[-1]

position_error = np.linalg.norm(
    current_state[:3] - final_waypoint[:3]
)
velocity_error = np.linalg.norm(
    current_state[3:] - final_waypoint[3:]
)

print("\n--- FINAL HOLDING POINT ---")
print(f"Position error: {position_error:.6f} m")
print(f"Velocity error: {velocity_error:.9f} m/s")

if (position_error <= POSITION_TOLERANCE
        and velocity_error <= VELOCITY_TOLERANCE):
    print("Final waypoint acceptance: PASS")
else:
    print("Final waypoint acceptance: FAIL")
