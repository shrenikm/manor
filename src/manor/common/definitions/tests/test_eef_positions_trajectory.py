"""
Round-trip tests for EEFPositionsTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_positions_trajectory import EEFPositionsTrajectory
from manor.common.definitions.tests.factories import random_eef_positions_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_positions_trajectory(rng)
    assert EEFPositionsTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_positions_trajectory(rng)
    assert EEFPositionsTrajectory.from_lcm_message(original.to_lcm_message()) == original
