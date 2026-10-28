
import numpy as np
from scipy.integrate import solve_ivp, trapezoid

from rendezvous.constants import circular_mean_motion
from rendezvous.twobody import MU_EARTH
from rendezvous.frames import lvlh_to_eci, eci_to_lvlh
from rendezvous.lqr import hcw_state_space, lqr_gain
from rendezvous.guidance import waypoint_acceptance_margin
from rendezvous.nonlinear_control import nonlinear_controlled_derivative
# 1. Mission configuration
n = circular_mean_motion(500_000)
radius = (MU_EARTH / n**2) ** (1.0 / 3.0)
speed = np.sqrt(MU_EARTH / radius)
end_time = 4 * (2 * np.pi / n)

A, B = hcw_state_space(n)

Q = np.diag([1e-6] * 3 + [1e-2] * 3)
R = np.eye(3) * 1e6
K = lqr_gain(A, B, Q, R)

MAX_ACCELERATION = 0.001
POSITION_TOLERANCE = 5.0
VELOCITY_TOLERANCE = 0.10
SCREENING_RANGE = 25.0

BASELINE = np.array(
    [100, -995, 0, 0, 0, 0], dtype=float
)

WAYPOINTS = [
    np.array([0, -200, 0, 0, 0, 0], dtype=float),
    np.array([0, -50, 0, 0, 0, 0], dtype=float),
    np.array([0, -30, 0, 0, 0, 0], dtype=float),
]


# 2. Convert combined ECI state into LVLH
def relative_lvlh(state):
    target = state[:6]
    chaser = state[6:]

    rho, rho_dot = eci_to_lvlh(
        target[:3], target[3:],
        chaser[:3], chaser[3:],
        n
    )

    return np.concatenate((rho, rho_dot))


# 3. Run one complete mission
def run_case(perturbation, max_acceleration=MAX_ACCELERATION):
    relative_initial = BASELINE + perturbation
    if np.linalg.norm(relative_initial[:3]) <= SCREENING_RANGE:
        raise RuntimeError("Initial state inside screening boundary")

    r_target = np.array([radius, 0.0, 0.0])
    v_target = np.array([0.0, speed, 0.0])

    r_chaser, v_chaser = lvlh_to_eci(
        r_target, v_target,
        relative_initial[:3],
        relative_initial[3:],
        n
    )

    current_state = np.concatenate((
        r_target, v_target, r_chaser, v_chaser
    ))

    current_time = 0.0
    total_delta_v = 0.0
    minimum_separation = np.linalg.norm(relative_initial[:3])

    for stage_number, waypoint in enumerate(WAYPOINTS, start=1):

        stop_at_waypoint = stage_number < 3

        def accepted(t, state):
            return waypoint_acceptance_margin(
                relative_lvlh(state),
                waypoint,
                POSITION_TOLERANCE,
                VELOCITY_TOLERANCE
            )

        accepted.terminal = True
        accepted.direction = -1

        def entered_buffer(t, state):
            return (
                np.linalg.norm(state[6:9] - state[:3])
                - SCREENING_RANGE
            )

        entered_buffer.terminal = True
        entered_buffer.direction = -1

        events = (
            [accepted, entered_buffer]
            if stop_at_waypoint
            else [entered_buffer]
        )

        
        solution = solve_ivp(
            lambda t, state: nonlinear_controlled_derivative(
                t,
                state,
                waypoint,
                n,
                K,
                max_acceleration
            ),
            (current_time, end_time),
            current_state,
            events=events,
            rtol=1e-10,
            atol=1e-10,
            max_step=5.0
        )


        if not solution.success:
            raise RuntimeError(solution.message)

        ranges = np.linalg.norm(
            solution.y[6:9] - solution.y[:3],
            axis=0
        )

        minimum_separation = min(
            minimum_separation, ranges.min()
        )

        if len(solution.t_events[-1]) > 0:
            raise RuntimeError(
                f"Stage {stage_number}: screening boundary reached"
            )

        
        if stop_at_waypoint and len(solution.t_events[0]) == 0:

            history = np.array([
                relative_lvlh(solution.y[:, i])
                for i in range(solution.t.size)
            ])

            position_errors = np.linalg.norm(
                history[:, :3] - waypoint[:3],
                axis=1
            )

            velocity_errors = np.linalg.norm(
                history[:, 3:] - waypoint[3:],
                axis=1
            )

            # Find the time when acceptance was closest
            normalized_errors = np.maximum(
                position_errors / POSITION_TOLERANCE,
                velocity_errors / VELOCITY_TOLERANCE
            )

            best_index = np.argmin(normalized_errors)

            requested = -(history - waypoint) @ K.T
            peak_requested = np.linalg.norm(
                requested, axis=1
            ).max()

            raise RuntimeError(
                f"Stage {stage_number}: waypoint not reached; "
                f"best position error={position_errors[best_index]:.3f} m; "
                f"best velocity error={velocity_errors[best_index]:.5f} m/s; "
                f"best normalized error={normalized_errors[best_index]:.3f}; "
                f"peak requested accel={peak_requested:.6f} m/s^2"
            )


        # Compute control effort
        history = np.array([
            relative_lvlh(solution.y[:, i])
            for i in range(solution.t.size)
        ])

        requested = -(history - waypoint) @ K.T

        
        actual_magnitudes = np.minimum(
            np.linalg.norm(requested, axis=1),
            max_acceleration
        )

        total_delta_v += trapezoid(
            actual_magnitudes,
            x=solution.t
        )

        current_time = solution.t[-1]
        current_state = solution.y[:, -1].copy()

    # 4. Final acceptance
    final_state = relative_lvlh(current_state)

    position_error = np.linalg.norm(
        final_state[:3] - WAYPOINTS[-1][:3]
    )

    velocity_error = np.linalg.norm(
        final_state[3:] - WAYPOINTS[-1][3:]
    )

    if waypoint_acceptance_margin(
        final_state,
        WAYPOINTS[-1],
        POSITION_TOLERANCE,
        VELOCITY_TOLERANCE
    ) > 0:
        raise RuntimeError("Final waypoint not accepted")

    return {
        "delta_v": float(total_delta_v),
        "min_separation": float(minimum_separation),
        "position_error": float(position_error),
        "velocity_error": float(velocity_error),
    }