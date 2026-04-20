"""
Round-trip tests for Action. Covers every variant field.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.action import Action
from manor.common.definitions.tests.factories import ACTION_VARIANT_FIELDS, random_action


@pytest.mark.parametrize("variant_field", ACTION_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_action(rng, variant_field)
    assert Action.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", ACTION_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_action(rng, variant_field)
    assert Action.from_lcm_message(original.to_lcm_message()) == original
