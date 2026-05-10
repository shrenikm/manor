"""
Round-trip tests for CartesianState.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.cartesian_state import CartesianState
from manor.common.definitions.tests.factories import random_cartesian_state
from manor.common.testing_utils import run_manor_tests


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_state(rng)
    assert CartesianState.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_cartesian_state(rng)
    assert CartesianState.from_lcm_message(original.to_lcm_message()) == original


if __name__ == "__main__":
    run_manor_tests()
