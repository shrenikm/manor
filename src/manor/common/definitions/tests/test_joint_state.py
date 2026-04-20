"""
Round-trip tests for JointState.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.joint_state import JointState
from manor.common.definitions.tests.factories import random_joint_state


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_state(rng)
    assert JointState.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_joint_state(rng)
    assert JointState.from_lcm_message(original.to_lcm_message()) == original
