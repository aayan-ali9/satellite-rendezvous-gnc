"""Week 1 sensitivity experiment; run python -m scripts.compare_initial_conditions."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from rendezvous.constants import circular_mean_motion, orbital_period
from rendezvous.hcw import analytical_uncontrolled, propagate_uncontrolled


def main() -> None:
    n = circular_mean_motion(500_000.0)
    times = np.linspace(0.0, 2 * orbital_period(n), 1201)
    cases = {
        "x0 = 100 m, vy0 = 0": np.array([100., -995., 0., 0., 0., 0.]),
        "x0 = 200 m, vy0 = 0 (baseline)": np.array([200., -995., 0., 0., 0., 0.]),
        "x0 = 200 m, vy0 = -2 n x0": np.array([200., -995., 0., 0., -2*n*200., 0.]),
    }
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    records = []
    for label, initial in cases.items():
        states = propagate_uncontrolled(initial, times, n)
        analytical = analytical_uncontrolled(initial, times, n)
        ranges = np.linalg.norm(states[:, :3], axis=1)
        axes[0].plot(times / 60, states[:, 0], label=label, linewidth=2)
        axes[1].plot(times / 60, states[:, 1], linewidth=2)
        axes[2].plot(times / 60, ranges, linewidth=2)
        records.append({
            "label": label,
            "initial_state_m_m_s": initial.tolist(),
            "final_state_m_m_s": states[-1].tolist(),
            "final_range_m": float(ranges[-1]),
            "max_position_difference_m": float(np.max(np.linalg.norm(
                states[:, :3] - analytical[:, :3], axis=1))),
            "max_velocity_difference_m_s": float(np.max(np.linalg.norm(
                states[:, 3:] - analytical[:, 3:], axis=1))),
        })
    for ax, ylabel in zip(axes, ["Radial x [m]", "Along-track y [m]", "Range [m]"]):
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=9, loc="upper right")
    axes[0].set_title("Initial-condition sensitivity | 500 km circular target | Zero thrust")
    axes[-1].set_xlabel("Elapsed time [min]")
    fig.tight_layout()
    output_dir = Path("results")
    output_dir.mkdir(parents=True, exist_ok=True)
    figure_path = output_dir / "initial_condition_comparison.png"
    fig.savefig(figure_path, dpi=180)
    plt.close(fig)
    summary_path = output_dir / "initial_condition_summary.json"
    summary_path.write_text(json.dumps({
        "altitude_m": 500_000.0, "mean_motion_rad_s": n,
        "duration_s": float(times[-1]), "sample_count": len(times),
        "solver": {"method": "RK45", "rtol": 1e-11, "atol": 1e-12},
        "cases": records,
    }, indent=2) + "\n", encoding="utf-8")
    for record in records:
        print(f"{record['label']}: final range = {record['final_range_m']:.3f} m")
    print(f"Saved {figure_path} and {summary_path}")


if __name__ == "__main__":
    main()
