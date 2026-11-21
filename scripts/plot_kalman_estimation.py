
"""Compare true HCW motion, noisy positions, and Kalman estimates."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rendezvous.constants import circular_mean_motion
from rendezvous.kalman import HCWKalmanFilter


def main():
    # Simulation settings
    dt = 10.0
    steps = 180
    measurement_std = 5.0

    rng = np.random.default_rng(42)
    n = circular_mean_motion(500_000)

    # True initial LVLH state
    true_state = np.array([
        100.0, -995.0, 10.0,
        0.02, -0.01, 0.005
    ])

    # Start the filter with an imperfect state estimate
    kf = HCWKalmanFilter(
        mean_motion=n,
        dt=dt,
        initial_state=true_state + np.array([
            10.0, -8.0, 6.0,
            0.15, -0.10, 0.10
        ]),
        initial_covariance=np.diag(
            [20.0**2] * 3 + [0.3**2] * 3
        ),
        position_noise_std=measurement_std,
        acceleration_noise_std=1e-5
    )

    # Store simulation results
    times = np.arange(steps + 1) * dt

    true_positions = np.zeros((steps + 1, 3))
    measurements = np.zeros((steps + 1, 3))
    estimates = np.zeros((steps + 1, 3))

    for i in range(steps + 1):
        if i > 0:
            # Propagate uncontrolled HCW motion
            true_state = kf.F @ true_state

            # Predict the next state
            kf.predict(np.zeros(3))

        # Generate noisy position measurement
        measured = (
            true_state[:3]
            + rng.normal(0.0, measurement_std, 3)
        )

        # Correct estimate using measurement
        estimated = kf.update(measured)

        true_positions[i] = true_state[:3]
        measurements[i] = measured
        estimates[i] = estimated[:3]

    # Compare position errors
    raw_error = np.linalg.norm(
        measurements - true_positions, axis=1
    )

    filter_error = np.linalg.norm(
        estimates - true_positions, axis=1
    )

    raw_rmse = np.sqrt(np.mean(raw_error**2))
    filter_rmse = np.sqrt(np.mean(filter_error**2))

    print(f"Raw position RMSE (3D): {raw_rmse:.2f} m")
    print(f"Kalman position RMSE (3D): {filter_rmse:.2f} m")

    # Plot position estimates
    fig, axes = plt.subplots(
        2, 2, figsize=(12, 8), sharex=True
    )

    t_min = times / 60.0
    labels = ["Radial x", "Along-track y", "Cross-track z"]

    for i, ax in enumerate(axes.flat[:3]):
        ax.plot(
            t_min, true_positions[:, i],
            label="True HCW", linewidth=2
        )

        ax.scatter(
            t_min, measurements[:, i],
            label="Noisy measurement",
            s=9, alpha=0.3
        )

        ax.plot(
            t_min, estimates[:, i],
            label="Kalman estimate",
            linestyle="--", linewidth=1.5
        )

        ax.set_title(labels[i])
        ax.set_ylabel("Relative position (m)")
        ax.grid(alpha=0.3)

    # Plot total position error
    ax = axes[1, 1]

    ax.plot(
        t_min, raw_error,
        label="Measurement error"
    )

    ax.plot(
        t_min, filter_error,
        label="Estimation error"
    )

    ax.set_title("3D position error magnitude")
    ax.set_ylabel("Position error (m)")
    ax.legend()
    ax.grid(alpha=0.3)

    for ax in axes[1, :]:
        ax.set_xlabel("Time (min)")

    handles, legend_labels = (
        axes[0, 0].get_legend_handles_labels()
    )

    fig.legend(
        handles, legend_labels,
        loc="upper center", ncol=3,
        bbox_to_anchor=(0.5, 0.99)
    )

    fig.suptitle(
        "HCW Kalman Filter: Position Estimation",
        y=1.03
    )
    fig.tight_layout()

    # Save the figure
    output = (
        Path(__file__).resolve().parents[1]
        / "figures"
        / "kalman_position_estimation.png"
    )

    output.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(
        output, dpi=200, bbox_inches="tight"
    )

    print(f"Saved figure: {output}")

    plt.show()


if __name__ == "__main__":
    main()
