"""First milestone: run and verify uncontrolled relative orbital motion.

From the repository root: python -m scripts.run_baseline
"""

from pathlib import Path
import json
import platform
import numpy as np
import scipy

from rendezvous.constants import circular_mean_motion, orbital_period
from rendezvous.hcw import propagate_uncontrolled, analytical_uncontrolled
from rendezvous.visualization import save_baseline_figures


def main() -> None:
    altitude_m = 500_000.0
    n = circular_mean_motion(altitude_m)
    period_s = orbital_period(n)

    # x = radial, y = along-track, z = orbit-normal; velocities in m/s.
    # ~1 km separation, initially at rest in the rotating target frame.
    initial_state = np.array([200.0, -995.0, 0.0, 0.0, 0.0, 0.0])
    times_s = np.linspace(0.0, 2.0 * period_s, 1201)

    states = propagate_uncontrolled(initial_state, times_s, n)
    closed_form = analytical_uncontrolled(initial_state, times_s, n)
    max_position_diff_m = np.max(np.linalg.norm(states[:, :3] - closed_form[:, :3], axis=1))
    max_velocity_diff_m_s = np.max(np.linalg.norm(states[:, 3:] - closed_form[:, 3:], axis=1))
    paths = save_baseline_figures(times_s, states, Path("results"))
    summary = {
        "model": "uncontrolled HCW; radial/along-track/orbit-normal LVLH",
        "altitude_m": altitude_m,
        "mean_motion_rad_s": n,
        "orbital_period_s": period_s,
        "initial_state_m_m_s": initial_state.tolist(),
        "final_state_m_m_s": states[-1].tolist(),
        "duration_s": float(times_s[-1] - times_s[0]),
        "sample_count": len(times_s),
        "solver": {"method": "RK45", "rtol": 1e-11, "atol": 1e-12},
        "initial_range_m": float(np.linalg.norm(initial_state[:3])),
        "final_range_m": float(np.linalg.norm(states[-1, :3])),
        "max_position_difference_m": float(max_position_diff_m),
        "max_velocity_difference_m_s": float(max_velocity_diff_m_s),
        "versions": {"python": platform.python_version(), "numpy": np.__version__,
                     "scipy": scipy.__version__},
    }
    summary_path = Path("results/baseline_summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Reference altitude: {altitude_m / 1000:.1f} km")
    print(f"Mean motion:        {n:.9f} rad/s")
    print(f"Orbital period:     {period_s / 60:.2f} minutes")
    print(f"Initial range:      {np.linalg.norm(initial_state[:3]):.3f} m")
    print(f"Final range:        {np.linalg.norm(states[-1, :3]):.3f} m")
    print(f"Max position difference, numerical vs analytical: {max_position_diff_m:.6e} m")
    print(f"Max velocity difference, numerical vs analytical: {max_velocity_diff_m_s:.6e} m/s")
    print("Saved figures:")
    for path in paths:
        print(f"  - {path}")
    print(f"Saved run parameters and verification metrics: {summary_path}")


if __name__ == "__main__":
    main()
