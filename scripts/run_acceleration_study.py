
import csv
from pathlib import Path

import numpy as np

from rendezvous.mission import run_case


# 1. Project directories
ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"

RESULTS.mkdir(exist_ok=True)


# 2. Load the previously failed initial condition
source = RESULTS / "robustness_sweep_extended.csv"

with source.open(newline="") as file:
    records = list(csv.DictReader(file))

failed_case = next(
    row for row in records
    if row["scenario"] == "extreme50_03"
)

perturbation = np.array([
    float(failed_case[key])
    for key in ["dx", "dy", "dz", "dvx", "dvy", "dvz"]
])


# 3. Acceleration limits to test
acceleration_limits = np.array([
    0.001000,
    0.001550,
    0.001575,
    0.002000
])


# 4. Run the sensitivity analysis
results = []

print("\n--- ACCELERATION AUTHORITY STUDY ---")

for limit in acceleration_limits:

    try:
        metrics = run_case(
            perturbation,
            max_acceleration=float(limit)
        )

        result = {
            "acceleration_limit": limit,
            "status": "PASS",
            "delta_v": metrics["delta_v"],
            "minimum_separation": metrics["min_separation"],
            "reason": ""
        }

        print(
            f"{limit:.6f} m/s^2: PASS | "
            f"delta-v={metrics['delta_v']:.6f} m/s"
        )

    except RuntimeError as error:

        result = {
            "acceleration_limit": limit,
            "status": "FAIL",
            "delta_v": "",
            "minimum_separation": "",
            "reason": str(error)
        }

        print(f"{limit:.6f} m/s^2: FAIL")

    results.append(result)


# 5. Save results
output = RESULTS / "acceleration_study_clean.csv"

with output.open("w", newline="") as file:

    writer = csv.DictWriter(
        file,
        fieldnames=[
            "acceleration_limit",
            "status",
            "delta_v",
            "minimum_separation",
            "reason"
        ]
    )

    writer.writeheader()
    writer.writerows(results)

print(f"\nResults saved to: {output}")
