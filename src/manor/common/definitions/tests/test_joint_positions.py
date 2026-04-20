"""
Round-trip tests for JointPositions.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.tests.factories import random_joint_positions


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_positions(rng)
    assert JointPositions.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_positions(rng)
    assert JointPositions.from_lcm_message(original.to_lcm_message()) == original
