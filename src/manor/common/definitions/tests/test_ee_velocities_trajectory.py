"""
Round-trip tests for EEVelocitiesTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.ee_velocities_trajectory import EEVelocitiesTrajectory
from manor.common.definitions.tests.factories import random_ee_velocities_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_velocities_trajectory(rng)
    assert EEVelocitiesTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_velocities_trajectory(rng)
    assert EEVelocitiesTrajectory.from_lcm_message(original.to_lcm_message()) == original
