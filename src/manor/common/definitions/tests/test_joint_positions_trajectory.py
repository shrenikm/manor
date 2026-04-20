"""
Round-trip tests for JointPositionsTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.joint_positions_trajectory import JointPositionsTrajectory
from manor.common.definitions.tests.factories import random_joint_positions_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_positions_trajectory(rng)
    assert JointPositionsTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_positions_trajectory(rng)
    assert JointPositionsTrajectory.from_lcm_message(original.to_lcm_message()) == original
