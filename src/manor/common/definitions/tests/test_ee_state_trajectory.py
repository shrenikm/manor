"""
Round-trip tests for EEStateTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.ee_state_trajectory import EEStateTrajectory
from manor.common.definitions.tests.factories import random_ee_state_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_state_trajectory(rng)
    assert EEStateTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_state_trajectory(rng)
    assert EEStateTrajectory.from_lcm_message(original.to_lcm_message()) == original
