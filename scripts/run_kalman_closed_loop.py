
"""Compare perfect-state and Kalman-estimated LQR rendezvous control.

Uses nonlinear two-body ECI propagation and a sampled, held LVLH command.
The original continuous-feedback mission is left unchanged.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

from rendezvous.control import saturate_acceleration
from rendezvous.frames import eci_to_lvlh, lvlh_basis, lvlh_to_eci
from rendezvous.guidance import waypoint_reached
from rendezvous.kalman import HCWKalmanFilter
from rendezvous.mission import (
    BASELINE, K, MAX_ACCELERATION, POSITION_TOLERANCE,
    SCREENING_RANGE, VELOCITY_TOLERANCE, WAYPOINTS,
    end_time, n, radius, speed,
)
from rendezvous.twobody import two_body_derivative


DT = 10.0  # Seconds between measurements and control updates
MEASUREMENT_STD = 5.0  # Position measurement noise in meters
SEED = 42


def relative_state(eci):
    """Convert the two ECI spacecraft states into relative LVLH."""
    rho, rho_dot = eci_to_lvlh(
        eci[:3], eci[3:6],
        eci[6:9], eci[9:12], n
    )
    return np.r_[rho, rho_dot]


def propagate(eci, command, t0, t1):
    """Propagate both spacecraft with a fixed LVLH control command."""

    def dynamics(t, state):
        target_dot = two_body_derivative(t, state[:6])
        chaser_dot = two_body_derivative(t, state[6:])

        # Convert the held LVLH acceleration into ECI
        C = lvlh_basis(state[:3], state[3:6])
        chaser_dot[3:] += C @ command

        return np.r_[target_dot, chaser_dot]

    def screening(t, state):
        separation = np.linalg.norm(
            state[6:9] - state[:3]
        )
        return separation - SCREENING_RANGE

    screening.terminal = True
    screening.direction = -1

    result = solve_ivp(
        dynamics,
        (t0, t1),
        eci,
        events=screening,
        max_step=5.0,
        rtol=1e-10,
        atol=1e-9,
    )

    if not result.success:
        raise RuntimeError(result.message)

    ranges = np.linalg.norm(
        result.y[6:9] - result.y[:3],
        axis=0
    )

    hit_boundary = len(result.t_events[0]) > 0

    return (
        result.y[:, -1],
        float(ranges.min()),
        hit_boundary,
        result.t[-1],
    )


def run_mission(use_kalman, seed=SEED):
    """Run one complete rendezvous with or without state estimation."""

    rng = np.random.default_rng(seed)

    # Initialize target and chaser in ECI
    target_r = np.array([radius, 0.0, 0.0])
    target_v = np.array([0.0, speed, 0.0])

    chaser_r, chaser_v = lvlh_to_eci(
        target_r,
        target_v,
        BASELINE[:3],
        BASELINE[3:],
        n,
    )

    eci = np.r_[
        target_r, target_v,
        chaser_r, chaser_v
    ]

    truth = relative_state(eci)

    # Initialize the Kalman filter
    kf = None

    if use_kalman:
        initial_error = np.array([
            10.0, -8.0, 6.0,
            0.05, -0.05, 0.05
        ])

        kf = HCWKalmanFilter(
            mean_motion=n,
            dt=DT,
            initial_state=BASELINE + initial_error,
            initial_covariance=np.diag(
                [20.0**2] * 3 + [0.3**2] * 3
            ),
            position_noise_std=MEASUREMENT_STD,
            acceleration_noise_std=1e-5,
        )

        # First noisy measurement
        measurement = (
            truth[:3]
            + rng.normal(0.0, MEASUREMENT_STD, 3)
        )
        kf.update(measurement)

    stage = 0
    t = 0.0
    delta_v = 0.0
    min_separation = np.linalg.norm(truth[:3])

    times = [t]
    states = [truth.copy()]
    estimates = [
        (kf.state if use_kalman else truth).copy()
    ]
    controls = [0.0]
    hit_boundary = False

    # Use complete 10-second sample intervals
    steps = int(end_time // DT)

    for _ in range(steps):

        # Controller uses either truth or the estimated state
        used_state = (
            kf.state if use_kalman else truth
        )

        # Check whether the current waypoint is reached
        if stage < 2 and waypoint_reached(
            used_state,
            WAYPOINTS[stage],
            POSITION_TOLERANCE,
            VELOCITY_TOLERANCE,
        ):
            stage += 1

        # Calculate LQR control using the selected state
        error = used_state - WAYPOINTS[stage]
        requested = -K @ error

        command = saturate_acceleration(
            requested,
            MAX_ACCELERATION,
        )

        # Propagate nonlinear orbital dynamics
        eci, step_min, hit_boundary, t_next = propagate(
            eci, command, t, t + DT
        )

        # Integrate applied acceleration over the step
        delta_v += (
            np.linalg.norm(command) * (t_next - t)
        )

        min_separation = min(
            min_separation, step_min
        )

        t = t_next
        truth = relative_state(eci)

        if use_kalman:
            # Predict with the acceleration actually applied
            kf.predict(command)

            # Measure relative position with simulated noise
            measurement = (
                truth[:3]
                + rng.normal(0.0, MEASUREMENT_STD, 3)
            )

            # Update the state estimate
            kf.update(measurement)

        times.append(t)
        states.append(truth.copy())

        estimates.append(
            (kf.state if use_kalman else truth).copy()
        )

        controls.append(np.linalg.norm(command))

        if hit_boundary:
            break

    # Evaluate final performance using the true state
    position_error = np.linalg.norm(
        truth[:3] - WAYPOINTS[2][:3]
    )

    velocity_error = np.linalg.norm(
        truth[3:] - WAYPOINTS[2][3:]
    )

    success = (
        not hit_boundary
        and stage == 2
        and position_error <= POSITION_TOLERANCE
        and velocity_error <= VELOCITY_TOLERANCE
    )

    return {
        "success": success,
        "boundary_hit": hit_boundary,
        "stage": stage + 1,
        "position_error": float(position_error),
        "velocity_error": float(velocity_error),
        "min_separation": float(min_separation),
        "delta_v": float(delta_v),
        "time": np.asarray(times),
        "true": np.asarray(states),
        "estimated": np.asarray(estimates),
        "acceleration": np.asarray(controls),
    }


def main():
    # Run both versions of the same mission
    baseline = run_mission(use_kalman=False)
    kalman = run_mission(use_kalman=True)

    for name, run in [
        ("Perfect-state", baseline),
        ("Kalman-estimated", kalman),
    ]:
        print(f"\n{name} control")
        print(f"  Final accepted: {run['success']}")
        print(f"  Last stage:     {run['stage']} / 3")
        print(f"  Position error: {run['position_error']:.3f} m")
        print(f"  Velocity error: {run['velocity_error']:.5f} m/s")
        print(f"  Min separation: {run['min_separation']:.3f} m")
        print(f"  Total delta-v:  {run['delta_v']:.3f} m/s")
        print(f"  Boundary hit:   {run['boundary_hit']}")

    # Kalman estimation accuracy
    position_error = np.linalg.norm(
        kalman["estimated"][:, :3]
        - kalman["true"][:, :3],
        axis=1,
    )

    velocity_error = np.linalg.norm(
        kalman["estimated"][:, 3:]
        - kalman["true"][:, 3:],
        axis=1,
    )

    position_rmse = np.sqrt(
        np.mean(position_error**2)
    )

    velocity_rmse = np.sqrt(
        np.mean(velocity_error**2)
    )

    print(f"\nKalman position RMSE: {position_rmse:.3f} m")
    print(f"Kalman velocity RMSE: {velocity_rmse:.5f} m/s")

    # Generate comparison plots
    fig, axs = plt.subplots(2, 2, figsize=(11, 8))

    for name, data in [
        ("Perfect state", baseline),
        ("Kalman estimate", kalman),
    ]:
        hours = data["time"] / 3600.0

        axs[0, 0].plot(
            data["true"][:, 1],
            data["true"][:, 0],
            label=name,
        )

        separation = np.linalg.norm(
            data["true"][:, :3],
            axis=1,
        )

        axs[0, 1].plot(
            hours, separation, label=name
        )

        axs[1, 1].plot(
            hours,
            data["acceleration"],
            label=name,
        )

    # Label mission waypoints
    for i, waypoint in enumerate(WAYPOINTS, 1):
        axs[0, 0].scatter(
            waypoint[1],
            waypoint[0],
            marker="x",
        )

        axs[0, 0].annotate(
            f"WP{i}",
            (waypoint[1], waypoint[0]),
            xytext=(5, 7 + 12 * (i - 1)),
            textcoords="offset points",
        )

    axs[0, 0].set(
        xlabel="Along-track y (m)",
        ylabel="Radial x (m)",
        title="Nonlinear rendezvous trajectory",
    )

    axs[0, 1].axhline(
        SCREENING_RANGE,
        linestyle=":",
        color="red",
        label="25 m screening",
    )

    axs[0, 1].set(
        xlabel="Time (h)",
        ylabel="Separation (m)",
        title="True target separation",
    )

    axs[1, 0].plot(
        kalman["time"] / 3600.0,
        position_error,
    )

    axs[1, 0].set(
        xlabel="Time (h)",
        ylabel="3D error (m)",
        title="Kalman position estimation error",
    )

    axs[1, 1].set(
        xlabel="Time (h)",
        ylabel="Acceleration (m/s²)",
        title="Applied control magnitude",
    )

    for ax in axs.flat:
        ax.grid(alpha=0.3)

    for ax in [axs[0, 0], axs[0, 1], axs[1, 1]]:
        ax.legend()

    fig.tight_layout()

    # Save the figure
    output = (
        Path(__file__).resolve().parents[1]
        / "figures"
        / "kalman_closed_loop.png"
    )

    output.parent.mkdir(
        parents=True, exist_ok=True
    )

    fig.savefig(output, dpi=180)

    print(f"\nSaved figure: {output}")
    plt.show()


if __name__ == "__main__":
    main()
