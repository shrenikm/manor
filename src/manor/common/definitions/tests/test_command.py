"""
Round-trip tests for Command. Covers every variant field.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.command import Command
from manor.common.definitions.tests.factories import COMMAND_VARIANT_FIELDS, random_command


@pytest.mark.parametrize("variant_field", COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_command(rng, variant_field)
    assert Command.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_command(rng, variant_field)
    assert Command.from_lcm_message(original.to_lcm_message()) == original
