"""
Round-trip + validator tests for JointTrajectoryCommand.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.joint_trajectory_command import JointTrajectoryCommand
from manor.common.definitions.tests.factories import (
    JOINT_TRAJECTORY_COMMAND_VARIANT_FIELDS,
    random_joint_positions_trajectory,
    random_joint_trajectory_command,
    random_joint_velocities_trajectory,
    random_timestamp_header,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError
from manor.common.testing_utils import run_manor_tests


@pytest.mark.parametrize("variant_field", JOINT_TRAJECTORY_COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_joint_trajectory_command(rng, variant_field)
    assert JointTrajectoryCommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", JOINT_TRAJECTORY_COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_joint_trajectory_command(rng, variant_field)
    assert JointTrajectoryCommand.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_variant_set() -> None:
    with pytest.raises(InvalidDefinitionError):
        JointTrajectoryCommand(header=TimestampHeader.construct_default())


def test_validator_rejects_both_variants_set(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        JointTrajectoryCommand(
            header=random_timestamp_header(rng),
            joint_positions_trajectory=random_joint_positions_trajectory(rng),
            joint_velocities_trajectory=random_joint_velocities_trajectory(rng),
        )


def test_construct_default_passes_validator() -> None:
    cmd = JointTrajectoryCommand.construct_default()
    assert cmd.joint_positions_trajectory is not None
    assert cmd.joint_velocities_trajectory is None


if __name__ == "__main__":
    run_manor_tests()
