
import numpy as np

from rendezvous.constants import circular_mean_motion
from rendezvous.lqr import hcw_state_space, lqr_gain

# Circular reference orbit at 500 km
n = circular_mean_motion(500_000)

# Build spacecraft dynamics
A, B = hcw_state_space(n)

# Initial illustrative LQR weights
Q = np.diag([1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2])
R = np.eye(3) * 1e6

# Calculate the feedback gain matrix
K = lqr_gain(A, B, Q, R)

print("LQR gain matrix K:")
print(K)
print("K dimensions:", K.shape)

# Inspect closed-loop stability
eigenvalues = np.linalg.eigvals(A - B @ K)
print("Closed-loop eigenvalues:")
print(eigenvalues)
