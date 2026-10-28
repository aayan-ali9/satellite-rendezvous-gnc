"""Simple, legible performance figures for the initial HCW model."""

from pathlib import Path
import matplotlib
matplotlib.use("Agg")  # Save graphics without needing a GUI window.
import matplotlib.pyplot as plt
import numpy as np


def save_baseline_figures(times_s: np.ndarray, states: np.ndarray,
                          output_dir: Path) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    out = []

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(states[:, 1], states[:, 0], linewidth=2, label="Uncontrolled chaser")
    ax.scatter([states[0, 1]], [states[0, 0]], marker="o", s=65, label="Start")
    ax.scatter([states[-1, 1]], [states[-1, 0]], marker="x", s=70, label="End")
    ax.scatter([0], [0], marker="*", s=110, label="Target")
    ax.set(xlabel="Along-track y [m]", ylabel="Radial x [m]",
           title="Uncontrolled Relative Motion — HCW Model")
    ax.grid(alpha=0.3)
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    path = output_dir / "baseline_relative_trajectory.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    out.append(path)

    range_m = np.linalg.norm(states[:, :3], axis=1)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times_s / 60.0, range_m, linewidth=2)
    ax.set(xlabel="Elapsed time [min]", ylabel="Relative distance [m]",
           title="Target Range Without Control")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = output_dir / "baseline_range.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    out.append(path)
    return out
