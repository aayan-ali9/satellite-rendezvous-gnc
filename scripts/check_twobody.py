
import numpy as np

from rendezvous.twobody import (
    MU_EARTH,
    two_body_derivative
)

# Example circular orbit at a radius of 6,878 km
radius = 6_878_000.0

# Circular-orbit velocity
speed = np.sqrt(MU_EARTH / radius)

state = np.array([
    radius, 0.0, 0.0,
    0.0, speed, 0.0
])

derivative = two_body_derivative(0.0, state)

print(f"Circular orbital speed: {speed:.6f} m/s")
print("Position derivative (m/s):")
print(derivative[:3])

print("Gravitational acceleration (m/s^2):")
print(derivative[3:])
