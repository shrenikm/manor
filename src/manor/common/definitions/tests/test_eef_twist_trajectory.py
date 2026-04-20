"""
Round-trip tests for EEFTwistTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_twist_trajectory import EEFTwistTrajectory
from manor.common.definitions.tests.factories import random_eef_twist_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_twist_trajectory(rng)
    assert EEFTwistTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_twist_trajectory(rng)
    assert EEFTwistTrajectory.from_lcm_message(original.to_lcm_message()) == original
