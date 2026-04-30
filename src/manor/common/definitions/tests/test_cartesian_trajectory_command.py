"""
Round-trip + validator tests for CartesianTrajectoryCommand.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.cartesian_trajectory_command import CartesianTrajectoryCommand
from manor.common.definitions.tests.factories import (
    CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FIELDS,
    random_cartesian_pose_trajectory,
    random_cartesian_trajectory_command,
    random_cartesian_twist_trajectory,
    random_timestamp_header,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError


@pytest.mark.parametrize("variant_field", CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_cartesian_trajectory_command(rng, variant_field)
    assert CartesianTrajectoryCommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_cartesian_trajectory_command(rng, variant_field)
    assert CartesianTrajectoryCommand.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_variant_set() -> None:
    with pytest.raises(InvalidDefinitionError):
        CartesianTrajectoryCommand(header=TimestampHeader.construct_default())


def test_validator_rejects_both_variants_set(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        CartesianTrajectoryCommand(
            header=random_timestamp_header(rng),
            cartesian_pose_trajectory=random_cartesian_pose_trajectory(rng),
            cartesian_twist_trajectory=random_cartesian_twist_trajectory(rng),
        )


def test_construct_default_passes_validator() -> None:
    cmd = CartesianTrajectoryCommand.construct_default()
    assert cmd.cartesian_pose_trajectory is not None
    assert cmd.cartesian_twist_trajectory is None
