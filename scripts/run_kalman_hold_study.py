
"""Compare final holding distances under noisy Kalman navigation."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt

from rendezvous.mission import SCREENING_RANGE
from scripts.run_kalman_closed_loop import run_mission


# Challenging case from the sensitivity analysis
SEED = 42
MEASUREMENT_NOISE = 10.0

HOLD_DISTANCES = [30.0, 40.0, 45.0]


def main():
    root = Path(__file__).resolve().parents[1]
    results = []

    for hold in HOLD_DISTANCES:

        # Change only the desired final holding distance
        run = run_mission(
            use_kalman=True,
            seed=SEED,
            measurement_std=MEASUREMENT_NOISE,
            final_hold_m=hold,
        )

        margin = run["min_separation"] - SCREENING_RANGE

        row = {
            "hold_distance_m": hold,
            "success": int(run["success"]),
            "boundary_hit": int(run["boundary_hit"]),
            "minimum_separation_m": run["min_separation"],
            "screening_margin_m": margin,
            "final_position_error_m": run["position_error"],
            "final_velocity_error_mps": run["velocity_error"],
            "delta_v_mps": run["delta_v"],
        }

        results.append(row)

        status = "PASS" if run["success"] else "FAIL"

        print(
            f"Hold: {hold:.0f} m | {status} | "
            f"Min separation: {run['min_separation']:.2f} m | "
            f"Margin: {margin:.2f} m | "
            f"Delta-v: {run['delta_v']:.3f} m/s",
            flush=True,
        )

    # Save numerical results
    csv_path = root / "results" / "kalman_hold_study.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file, fieldnames=results[0].keys()
        )
        writer.writeheader()
        writer.writerows(results)

    # Plot closest approach against selected hold distance
    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(
        HOLD_DISTANCES,
        [r["minimum_separation_m"] for r in results],
        marker="o",
        label="Minimum true separation",
    )

    ax.axhline(
        SCREENING_RANGE,
        linestyle="--",
        label="25 m screening boundary",
    )

    for row in results:
        label = "PASS" if row["success"] else "FAIL"

        ax.annotate(
            label,
            (
                row["hold_distance_m"],
                row["minimum_separation_m"],
            ),
            xytext=(5, 8),
            textcoords="offset points",
        )

    ax.set_xlabel("Desired final holding distance (m)")
    ax.set_ylabel("Minimum true separation (m)")
    ax.set_title("Kalman Rendezvous: Holding Distance Study")
    ax.set_xticks(HOLD_DISTANCES)
    ax.legend()
    ax.grid(alpha=0.3)

    fig.tight_layout()

    figure_path = root / "figures" / "kalman_hold_study.png"
    figure_path.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(figure_path, dpi=180)

    print(f"\nResults saved: {csv_path}")
    print(f"Figure saved: {figure_path}")

    plt.show()


if __name__ == "__main__":
    main()
