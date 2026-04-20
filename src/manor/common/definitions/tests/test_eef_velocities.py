"""
Round-trip tests for EEFVelocities.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.tests.factories import random_eef_velocities


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_velocities(rng)
    assert EEFVelocities.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_eef_velocities(rng)
    assert EEFVelocities.from_lcm_message(original.to_lcm_message()) == original
