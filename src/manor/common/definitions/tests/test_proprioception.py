"""
Round-trip tests for Proprioception.

Exercises both ends of the optional-field space: every optional populated and
every optional left as None.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.tests.factories import random_proprioception


@pytest.mark.parametrize("all_none", [True, False], ids=["bare", "full"])
def test_capnp_roundtrip(rng: np.random.Generator, all_none: bool) -> None:
    original = random_proprioception(rng, all_none=all_none)
    assert Proprioception.deserialize(original.serialize()) == original


@pytest.mark.parametrize("all_none", [True, False], ids=["bare", "full"])
def test_lcm_roundtrip(rng: np.random.Generator, all_none: bool) -> None:
    original = random_proprioception(rng, all_none=all_none)
    assert Proprioception.from_lcm_message(original.to_lcm_message()) == original
