"""
Round-trip tests for CartesianPose.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.tests.factories import random_cartesian_pose
from manor.common.testing_utils import run_manor_tests


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_pose(rng)
    assert CartesianPose.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_pose(rng)
    assert CartesianPose.from_lcm_message(original.to_lcm_message()) == original


if __name__ == "__main__":
    run_manor_tests()
