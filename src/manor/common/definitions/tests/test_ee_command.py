"""
Round-trip + validator tests for EECommand.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.tests.factories import (
    EE_COMMAND_VARIANT_FIELDS,
    random_ee_command,
    random_ee_positions,
    random_ee_velocities,
    random_timestamp_header,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError


@pytest.mark.parametrize("variant_field", EE_COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_ee_command(rng, variant_field)
    assert EECommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", EE_COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_ee_command(rng, variant_field)
    assert EECommand.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_variant_set() -> None:
    with pytest.raises(InvalidDefinitionError):
        EECommand(header=TimestampHeader.construct_default())


def test_validator_rejects_both_variants_set(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        EECommand(
            header=random_timestamp_header(rng),
            ee_positions=random_ee_positions(rng),
            ee_velocities=random_ee_velocities(rng),
        )


def test_construct_default_passes_validator() -> None:
    cmd = EECommand.construct_default()
    assert cmd.ee_positions is not None
    assert cmd.ee_velocities is None
