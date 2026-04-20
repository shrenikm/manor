"""
Round-trip tests for EEFStateTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_state_trajectory import EEFStateTrajectory
from manor.common.definitions.tests.factories import random_eef_state_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_state_trajectory(rng)
    assert EEFStateTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_state_trajectory(rng)
    assert EEFStateTrajectory.from_lcm_message(original.to_lcm_message()) == original
