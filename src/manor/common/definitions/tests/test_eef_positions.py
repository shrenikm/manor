"""
Round-trip tests for EEFPositions.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.tests.factories import random_eef_positions


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_positions(rng)
    assert EEFPositions.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_positions(rng)
    assert EEFPositions.from_lcm_message(original.to_lcm_message()) == original
