
import numpy as np

from rendezvous.frames import (
    lvlh_basis,
    lvlh_to_eci,
    eci_to_lvlh
)
# Target's position and velocity in ECI
r_target = np.array([0.0, 6878000.0, 0.0])
v_target = np.array([-7600.0, 0.0, 0.0])


# Chaser's position and velocity relative to target in LVLH
rho = np.array([100.0, 0.0, 0.0])
rho_dot = np.array([0.0, 0.0, 0.0])

# Approximate orbital angular velocity
n = 0.001107

# Calculate the rotation matrix
C = lvlh_basis(r_target, v_target)

# Convert chaser state from LVLH to ECI
r_chaser, v_chaser = lvlh_to_eci(
    r_target, v_target, rho, rho_dot, n
)

print("LVLH-to-ECI rotation matrix:")
print(C)

print("\nChaser ECI position (m):")
print(r_chaser)

print("\nChaser ECI velocity (m/s):")
print(v_chaser)

# Convert the calculated ECI state back into LVLH
rho_recovered, rho_dot_recovered = eci_to_lvlh(
    r_target,
    v_target,
    r_chaser,
    v_chaser,
    n
)

print("\n--- ROUND-TRIP CHECK ---")
print("Recovered LVLH position:", rho_recovered)
print("Recovered LVLH velocity:", rho_dot_recovered)

print(
    "Position round-trip error:",
    np.linalg.norm(rho_recovered - rho)
)

print(
    "Velocity round-trip error:",
    np.linalg.norm(rho_dot_recovered - rho_dot)
)
