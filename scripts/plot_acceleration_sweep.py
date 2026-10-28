
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# 1. Locate project files
ROOT = Path(__file__).resolve().parents[1]

results_dir = ROOT / "results"
figures_dir = ROOT / "figures"

figures_dir.mkdir(exist_ok=True)

data_files = [
    results_dir / "acceleration_sweep_corrected.csv",
    results_dir / "acceleration_sweep_fine.csv"
]


# 2. Read and combine the CSV results
records = {}

for path in data_files:

    if not path.exists():
        raise FileNotFoundError(
            f"Could not find {path}"
        )

    with path.open(newline="") as file:

        reader = csv.DictReader(file)

        for row in reader:

            acceleration = float(
                row["acceleration_limit"]
            )

            status = row["status"].strip().upper()

            delta_v = (
                float(row["delta_v"])
                if row["delta_v"]
                else np.nan
            )

            # Repeated acceleration values are overwritten
            # by the later (fine-sweep) file.
            records[acceleration] = (
                status,
                delta_v
            )


# 3. Separate successful and failed tests
accelerations = np.array(sorted(records))

statuses = np.array([
    records[a][0] for a in accelerations
])

delta_vs = np.array([
    records[a][1] for a in accelerations
])

passed = statuses == "PASS"
failed = statuses == "FAIL"

# Convert m/s^2 to mm/s^2 for readable axes
accel_display = accelerations * 1000


# 4. Plot mission success versus acceleration
fig1, ax1 = plt.subplots(figsize=(10, 5))

ax1.scatter(
    accel_display[failed],
    np.zeros(np.count_nonzero(failed)),
    marker="x",
    s=100,
    label="Failed mission"
)

ax1.scatter(
    accel_display[passed],
    np.ones(np.count_nonzero(passed)),
    marker="o",
    s=70,
    label="Successful mission"
)

ax1.set_yticks([0, 1])
ax1.set_yticklabels(["FAIL", "PASS"])

ax1.set_xlabel(
    "Maximum control acceleration (mm/s²)"
)

ax1.set_ylabel("Mission result")

ax1.set_title(
    "Rendezvous Success vs. Acceleration Authority"
)

ax1.grid(True, alpha=0.3)
ax1.legend()

fig1.tight_layout()

fig1.savefig(
    figures_dir / "acceleration_success.png",
    dpi=300
)


# 5. Plot maneuver delta-v for successful missions
fig2, ax2 = plt.subplots(figsize=(10, 5))

ax2.plot(
    accel_display[passed],
    delta_vs[passed],
    marker="o",
    linewidth=2
)

ax2.set_xlabel(
    "Maximum control acceleration (mm/s²)"
)

ax2.set_ylabel("Total maneuver Δv (m/s)")

ax2.set_title(
    "Maneuver Delta-v vs. Acceleration Authority"
)

ax2.grid(True, alpha=0.3)

fig2.tight_layout()

fig2.savefig(
    figures_dir / "acceleration_delta_v.png",
    dpi=300
)


# 6. Summarize the sampled transition
print("--- ACCELERATION SENSITIVITY RESULTS ---")

print(f"Acceleration limits tested: {len(accelerations)}")
print(f"Successful cases: {np.count_nonzero(passed)}")
print(f"Failed cases: {np.count_nonzero(failed)}")

if np.any(passed) and np.any(failed):

    highest_failure = accelerations[failed].max()
    lowest_success = accelerations[passed].min()

    print(
        f"Highest tested failure: "
        f"{highest_failure:.6f} m/s^2"
    )

    print(
        f"Lowest tested success: "
        f"{lowest_success:.6f} m/s^2"
    )

print("\nFigures saved to:", figures_dir)

plt.show()
