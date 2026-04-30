"""
Round-trip tests for Command. Covers every combination of joint command
variant against either no EE command or each EE command variant.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.command import Command
from manor.common.definitions.tests.factories import (
    EE_COMMAND_VARIANT_FIELDS,
    JOINT_COMMAND_VARIANT_FIELDS,
    random_command,
)


_EE_FIELD_OPTIONS: tuple[str | None, ...] = (None, *EE_COMMAND_VARIANT_FIELDS)


@pytest.mark.parametrize("joint_variant_field", JOINT_COMMAND_VARIANT_FIELDS)
@pytest.mark.parametrize("ee_variant_field", _EE_FIELD_OPTIONS)
def test_capnp_roundtrip(rng: np.random.Generator, joint_variant_field: str, ee_variant_field: str | None) -> None:
    original = random_command(rng, joint_variant_field, ee_variant_field)
    assert Command.deserialize(original.serialize()) == original


@pytest.mark.parametrize("joint_variant_field", JOINT_COMMAND_VARIANT_FIELDS)
@pytest.mark.parametrize("ee_variant_field", _EE_FIELD_OPTIONS)
def test_lcm_roundtrip(rng: np.random.Generator, joint_variant_field: str, ee_variant_field: str | None) -> None:
    original = random_command(rng, joint_variant_field, ee_variant_field)
    assert Command.from_lcm_message(original.to_lcm_message()) == original
