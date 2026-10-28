
import numpy as np
import pytest

from rendezvous.mission import run_case


# Previously identified extreme initial condition
EXTREME_PERTURBATION = np.array([
    864.3488222027962,
    -493.4285392141187,
    -61.31946148527434,
    0.388668788030872,
    0.2173830372330932,
    -0.09403901780752481
])


def test_extreme_case_fails_at_baseline_acceleration():
    """
    The original acceleration authority is insufficient
    for this scenario under our current mission constraints.
    """

    with pytest.raises(
        RuntimeError,
        match="Stage 1: waypoint not reached"
    ):
        run_case(
            EXTREME_PERTURBATION,
            max_acceleration=0.001
        )


def test_extreme_case_passes_at_higher_acceleration():
    """
    Regression test based on our previously verified
    nonlinear rendezvous simulation.
    """

    result = run_case(
        EXTREME_PERTURBATION,
        max_acceleration=0.002
    )

    # Verify the previously established maneuver delta-v
    assert result["delta_v"] == pytest.approx(
        10.816088,
        abs=0.02
    )

    # Verify simulated separation and final acceptance
    assert result["min_separation"] >= 25.0

    assert result["position_error"] <= 5.0

    assert result["velocity_error"] <= 0.10
