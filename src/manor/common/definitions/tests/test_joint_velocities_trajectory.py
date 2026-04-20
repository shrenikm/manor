"""
Round-trip tests for JointVelocitiesTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.joint_velocities_trajectory import JointVelocitiesTrajectory
from manor.common.definitions.tests.factories import random_joint_velocities_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_velocities_trajectory(rng)
    assert JointVelocitiesTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_velocities_trajectory(rng)
    assert JointVelocitiesTrajectory.from_lcm_message(original.to_lcm_message()) == original
