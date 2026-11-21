
"""Compare successful and failed Kalman rendezvous cases."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rendezvous.mission import SCREENING_RANGE
from scripts.run_kalman_closed_loop import run_mission


def main():
    seed = 42

    # Same random seed, different measurement noise
    cases = [
        ("5 m noise", 5.0),
        ("10 m noise", 10.0),
    ]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    for label, noise in cases:
        result = run_mission(
            use_kalman=True,
            seed=seed,
            measurement_std=noise,
            initial_error_scale=1.0,
        )

        time = result["time"] / 3600.0

        # True and estimated spacecraft separation
        true_range = np.linalg.norm(
            result["true"][:, :3], axis=1
        )

        estimated_range = np.linalg.norm(
            result["estimated"][:, :3], axis=1
        )

        range_error = estimated_range - true_range

        # Focus on the close-range portion of the mission
        close = true_range < 100.0

        axes[0].plot(
            time[close],
            true_range[close],
            label=label,
        )

        axes[1].plot(
            time[close],
            range_error[close],
            label=label,
        )

        if result["boundary_hit"]:
            axes[0].scatter(
                time[-1],
                true_range[-1],
                marker="x",
                s=100,
                label=f"{label} boundary event",
            )

        print(f"\n{label}")
        print(f"Success: {result['success']}")
        print(f"Boundary hit: {result['boundary_hit']}")
        print(f"Minimum separation: {result['min_separation']:.3f} m")
        print(f"Final time: {time[-1]:.3f} h")
        print(f"Final estimated range: {estimated_range[-1]:.3f} m")
        print(f"Final true range: {true_range[-1]:.3f} m")
        print(f"Final range estimation error: {range_error[-1]:.3f} m")

    # True spacecraft separation
    axes[0].axhline(
        SCREENING_RANGE,
        linestyle="--",
        color="red",
        label="25 m screening boundary",
    )

    axes[0].axhline(
        30.0,
        linestyle=":",
        color="gray",
        label="30 m holding distance",
    )

    axes[0].set_ylabel("True separation (m)")
    axes[0].set_title("Final Approach: Measurement Noise Comparison")
    axes[0].legend()

    # Estimated separation error
    axes[1].axhline(0.0, linestyle="--", color="gray")
    axes[1].set_ylabel("Estimated - true range (m)")
    axes[1].set_xlabel("Simulation time (hours)")
    axes[1].set_title("Separation Estimation Error")
    axes[1].legend()

    for ax in axes:
        ax.grid(alpha=0.3)

    fig.tight_layout()

    output = (
        Path(__file__).resolve().parents[1]
        / "figures"
        / "kalman_boundary_diagnostic.png"
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)

    print(f"\nFigure saved: {output}")

    plt.show()


if __name__ == "__main__":
    main()
