"""
Round-trip tests for CartesianPoseTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.cartesian_pose_trajectory import CartesianPoseTrajectory
from manor.common.definitions.tests.factories import random_cartesian_pose_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_pose_trajectory(rng)
    assert CartesianPoseTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_pose_trajectory(rng)
    assert CartesianPoseTrajectory.from_lcm_message(original.to_lcm_message()) == original
