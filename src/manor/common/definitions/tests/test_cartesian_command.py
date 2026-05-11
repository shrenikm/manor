"""
Round-trip + validator tests for CartesianCommand.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.tests.factories import (
    CARTESIAN_COMMAND_VARIANT_FIELDS,
    random_cartesian_command,
    random_cartesian_pose,
    random_cartesian_twist,
    random_timestamp_header,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError
from manor.common.testing_utils import run_manor_tests


@pytest.mark.parametrize("variant_field", CARTESIAN_COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_cartesian_command(rng, variant_field)
    assert CartesianCommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", CARTESIAN_COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_cartesian_command(rng, variant_field)
    assert CartesianCommand.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_variant_set() -> None:
    with pytest.raises(InvalidDefinitionError):
        CartesianCommand(header=TimestampHeader.construct_default())


def test_validator_rejects_both_variants_set(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        CartesianCommand(
            header=random_timestamp_header(rng),
            cartesian_pose=random_cartesian_pose(rng),
            cartesian_twist=random_cartesian_twist(rng),
        )


def test_construct_default_passes_validator() -> None:
    cmd = CartesianCommand.construct_default()
    assert cmd.cartesian_pose is not None
    assert cmd.cartesian_twist is None


if __name__ == "__main__":
    run_manor_tests()
