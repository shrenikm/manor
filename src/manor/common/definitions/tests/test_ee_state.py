"""
Round-trip tests for EEState.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.ee_state import EEState
from manor.common.definitions.tests.factories import random_ee_state
from manor.common.testing_utils import run_manor_tests


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_state(rng)
    assert EEState.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_ee_state(rng)
    assert EEState.from_lcm_message(original.to_lcm_message()) == original


if __name__ == "__main__":
    run_manor_tests()
