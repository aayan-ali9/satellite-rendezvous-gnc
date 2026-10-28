
import csv
from pathlib import Path
import numpy as np

from rendezvous.mission import run_case



# 5. Generate reproducible scenarios
rng = np.random.default_rng(42)

position_offset = np.array([10, -10, 5, 0, 0, 0])
velocity_offset = np.array([0, 0, 0, 0.01, -0.01, 0.005])

scenarios = [
    ("nominal", np.zeros(6)),
    ("position_only", position_offset),
    ("velocity_only", velocity_offset),
    ("combined", position_offset + velocity_offset),
]

# Standard deviations for illustrative random variations
scales = np.array([10, 10, 5, 0.01, 0.01, 0.005])

for i in range(8):
    perturbation = rng.normal(0.0, scales)
    scenarios.append((f"random_{i+1:02d}", perturbation))


# Additional robustness stress tests

for group_name, multiplier in [
    ("moderate", 3.0),
    ("severe", 10.0),
    ("extreme20", 20.0),
    ("extreme30", 30.0),
    ("extreme50", 50.0)
]:

    for i in range(8):
        perturbation = rng.normal(
            0.0,
            scales * multiplier
        )

        scenarios.append((
            f"{group_name}_{i+1:02d}",
            perturbation
        ))

# 6. Execute all scenarios
results = []

for name, perturbation in scenarios:
    row = {
        "scenario": name,
        "dx": perturbation[0],
        "dy": perturbation[1],
        "dz": perturbation[2],
        "dvx": perturbation[3],
        "dvy": perturbation[4],
        "dvz": perturbation[5],
    }

    try:
        metrics = run_case(perturbation)

        row.update(metrics)
        row["status"] = "PASS"
        row["reason"] = ""

        print(
            f"{name:<15} PASS  "
            f"delta-v={metrics['delta_v']:.6f} m/s  "
            f"min-range={metrics['min_separation']:.3f} m"
        )

    except RuntimeError as error:
        row["status"] = "FAIL"
        row["reason"] = str(error)

        print(f"{name:<15} FAIL  {error}")

    results.append(row)


# 7. Save results to CSV
output_dir = Path("results")
output_dir.mkdir(exist_ok=True)

csv_path = output_dir / "robustness_sweep_refactored.csv"

columns = [
    "scenario", "dx", "dy", "dz",
    "dvx", "dvy", "dvz",
    "status", "delta_v", "min_separation",
    "position_error", "velocity_error", "reason"
]

with csv_path.open("w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=columns)
    writer.writeheader()
    writer.writerows(results)


# 8. Summary
passed = [r for r in results if r["status"] == "PASS"]

print("\n--- ROBUSTNESS SWEEP SUMMARY ---")
print(f"Scenarios tested: {len(results)}")
print(f"Successful missions: {len(passed)}")
print(f"Failed missions: {len(results) - len(passed)}")
print(f"Success rate: {100 * len(passed) / len(results):.1f}%")

if passed:
    print(
        f"Maximum successful delta-v: "
        f"{max(r['delta_v'] for r in passed):.6f} m/s"
    )
    print(
        f"Smallest successful separation: "
        f"{min(r['min_separation'] for r in passed):.3f} m"
    )

print(f"Results saved to: {csv_path}")


print("\n--- SUCCESS RATE BY SEVERITY ---")

for group_name in [
    "random", "moderate", "severe",
    "extreme20", "extreme30", "extreme50"
]:

    group = [
        row for row in results
        if row["scenario"].startswith(group_name + "_")
    ]
    if not group:
        continue
    successes = sum(
        row["status"] == "PASS"
        for row in group
    )

    rate = 100 * successes / len(group)

    print(
        f"{group_name:<12}: "
        f"{successes}/{len(group)} passed "
        f"({rate:.1f}%)"
    )
