
"""Evaluate Kalman rendezvous sensitivity to estimation errors."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rendezvous.mission import SCREENING_RANGE
from scripts.run_kalman_closed_loop import run_mission


# Reproducible simulation settings
SEEDS = [41, 42]

SCENARIOS = [
    ("Baseline", 5.0, 1.0),
    ("Higher noise", 10.0, 1.0),
    ("Larger initial error", 5.0, 2.0),
]


def main():
    root = Path(__file__).resolve().parents[1]

    csv_path = root / "results" / "kalman_sensitivity.csv"
    figure_path = root / "figures" / "kalman_sensitivity.png"

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    figure_path.parent.mkdir(parents=True, exist_ok=True)

    results = []

    for scenario, noise_std, error_scale in SCENARIOS:
        for seed in SEEDS:

            # Run nonlinear rendezvous with estimated states
            run = run_mission(
                use_kalman=True,
                seed=seed,
                measurement_std=noise_std,
                initial_error_scale=error_scale,
            )

            # Compare estimated and true states
            position_error = (
                run["estimated"][:, :3]
                - run["true"][:, :3]
            )

            velocity_error = (
                run["estimated"][:, 3:]
                - run["true"][:, 3:]
            )

            position_rmse = np.sqrt(
                np.mean(np.sum(position_error**2, axis=1))
            )

            velocity_rmse = np.sqrt(
                np.mean(np.sum(velocity_error**2, axis=1))
            )

            row = {
                "scenario": scenario,
                "seed": seed,
                "measurement_noise_m": noise_std,
                "initial_error_scale": error_scale,
                "success": int(run["success"]),
                "boundary_hit": int(run["boundary_hit"]),
                "stage_reached": run["stage"],
                "minimum_separation_m": run["min_separation"],
                "final_position_error_m": run["position_error"],
                "final_velocity_error_mps": run["velocity_error"],
                "delta_v_mps": run["delta_v"],
                "position_rmse_m": float(position_rmse),
                "velocity_rmse_mps": float(velocity_rmse),
                "duration_hours": float(run["time"][-1] / 3600),
            }

            results.append(row)

            status = "PASS" if run["success"] else "FAIL"

            print(
                f"{scenario} | Seed {seed} | {status} | "
                f"Min separation: {run['min_separation']:.2f} m | "
                f"Position RMSE: {position_rmse:.2f} m",
                flush=True,
            )

    # Save results to CSV
    with csv_path.open(
        "w", newline="", encoding="utf-8"
    ) as file:

        writer = csv.DictWriter(
            file, fieldnames=results[0].keys()
        )

        writer.writeheader()
        writer.writerows(results)

    # Create sensitivity plots
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    for i, (scenario, _, _) in enumerate(SCENARIOS):

        group = [
            row for row in results
            if row["scenario"] == scenario
        ]

        x = i + np.linspace(-0.1, 0.1, len(group))

        axes[0].scatter(
            x,
            [row["position_rmse_m"] for row in group],
        )

        axes[1].scatter(
            x,
            [row["minimum_separation_m"] for row in group],
        )

    axes[0].set_title("Position Estimation Accuracy")
    axes[0].set_ylabel("3D Position RMSE (m)")

    axes[1].set_title("Minimum Spacecraft Separation")
    axes[1].set_ylabel("Separation (m)")

    axes[1].axhline(
        SCREENING_RANGE,
        linestyle="--",
        label="25 m screening boundary",
    )

    axes[1].legend()

    for ax in axes:
        ax.set_xticks(range(len(SCENARIOS)))
        ax.set_xticklabels(
            [item[0] for item in SCENARIOS],
            rotation=15,
        )
        ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(figure_path, dpi=180)

    # Summary of completed simulations
    print("\nKalman Sensitivity Analysis")

    for scenario, _, _ in SCENARIOS:

        group = [
            row for row in results
            if row["scenario"] == scenario
        ]

        successes = sum(row["success"] for row in group)
        boundary_hits = sum(
            row["boundary_hit"] for row in group
        )

        print(
            f"{scenario}: "
            f"{successes}/{len(group)} successful, "
            f"{boundary_hits} boundary hits"
        )

    print(f"\nResults saved: {csv_path}")
    print(f"Figure saved: {figure_path}")

    plt.show()


if __name__ == "__main__":
    main()
