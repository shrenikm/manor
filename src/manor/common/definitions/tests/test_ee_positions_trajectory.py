"""
Round-trip tests for EEPositionsTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.ee_positions_trajectory import EEPositionsTrajectory
from manor.common.definitions.tests.factories import random_ee_positions_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_positions_trajectory(rng)
    assert EEPositionsTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_positions_trajectory(rng)
    assert EEPositionsTrajectory.from_lcm_message(original.to_lcm_message()) == original
