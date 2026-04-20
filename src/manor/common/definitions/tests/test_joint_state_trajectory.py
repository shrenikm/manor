"""
Round-trip tests for JointStateTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.joint_state_trajectory import JointStateTrajectory
from manor.common.definitions.tests.factories import random_joint_state_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_state_trajectory(rng)
    assert JointStateTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_state_trajectory(rng)
    assert JointStateTrajectory.from_lcm_message(original.to_lcm_message()) == original
