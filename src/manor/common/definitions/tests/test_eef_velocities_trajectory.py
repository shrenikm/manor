"""
Round-trip tests for EEFVelocitiesTrajectory.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_velocities_trajectory import EEFVelocitiesTrajectory
from manor.common.definitions.tests.factories import random_eef_velocities_trajectory


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_velocities_trajectory(rng)
    assert EEFVelocitiesTrajectory.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_velocities_trajectory(rng)
    assert EEFVelocitiesTrajectory.from_lcm_message(original.to_lcm_message()) == original
