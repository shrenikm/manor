"""
Round-trip tests for EEPositions.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.tests.factories import random_ee_positions


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_positions(rng)
    assert EEPositions.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_positions(rng)
    assert EEPositions.from_lcm_message(original.to_lcm_message()) == original
