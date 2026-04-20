"""
Round-trip tests for EEFPose.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.tests.factories import random_eef_pose


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_pose(rng)
    assert EEFPose.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_pose(rng)
    assert EEFPose.from_lcm_message(original.to_lcm_message()) == original
