"""
Round-trip tests for CartesianStateTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.cartesian_state_trajectory import CartesianStateTrajectory
from manor.common.definitions.tests.factories import random_cartesian_state_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_state_trajectory(rng)
    assert CartesianStateTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_state_trajectory(rng)
    assert CartesianStateTrajectory.from_lcm_message(original.to_lcm_message()) == original
