"""Earth constants and reference orbit quantities (SI units throughout)."""

import math

EARTH_MU_M3_S2 = 3.986004418e14  # Earth gravitational parameter, m^3/s^2
EARTH_RADIUS_M = 6_378_137.0  # Equatorial radius, m (spherical approximation)


def circular_mean_motion(altitude_m: float) -> float:
    """Return circular-orbit mean motion n (rad/s) at altitude above model radius."""
    if altitude_m < 0:
        raise ValueError("Altitude must be nonnegative")
    radius_m = EARTH_RADIUS_M + altitude_m
    return math.sqrt(EARTH_MU_M3_S2 / radius_m**3)


def orbital_period(n_rad_s: float) -> float:
    """Return circular-orbit period in seconds."""
    if n_rad_s <= 0:
        raise ValueError("Mean motion must be positive")
    return 2.0 * math.pi / n_rad_s
