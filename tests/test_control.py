
import numpy as np

from rendezvous.control import saturate_acceleration


def test_acceleration_within_limit():
    requested = np.array([0.0003, 0.0004, 0.0])

    actual = saturate_acceleration(requested, 0.001)

    np.testing.assert_allclose(actual, requested)


def test_acceleration_saturation():
    requested = np.array([0.0008, 0.0008, 0.0])

    actual = saturate_acceleration(requested, 0.001)

    assert np.isclose(np.linalg.norm(actual), 0.001)

    np.testing.assert_allclose(
        actual,
        requested * (0.001 / np.linalg.norm(requested))
    )


def test_zero_acceleration():
    requested = np.zeros(3)

    actual = saturate_acceleration(requested, 0.001)

    np.testing.assert_array_equal(actual, requested)
