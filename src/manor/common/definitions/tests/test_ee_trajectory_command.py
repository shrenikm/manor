"""
Round-trip + validator tests for EETrajectoryCommand.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.ee_trajectory_command import EETrajectoryCommand
from manor.common.definitions.tests.factories import (
    EE_TRAJECTORY_COMMAND_VARIANT_FIELDS,
    random_ee_positions_trajectory,
    random_ee_trajectory_command,
    random_ee_velocities_trajectory,
    random_timestamp_header,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError
from manor.common.testing_utils import run_manor_tests


@pytest.mark.parametrize("variant_field", EE_TRAJECTORY_COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_ee_trajectory_command(rng, variant_field)
    assert EETrajectoryCommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", EE_TRAJECTORY_COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_ee_trajectory_command(rng, variant_field)
    assert EETrajectoryCommand.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_variant_set() -> None:
    with pytest.raises(InvalidDefinitionError):
        EETrajectoryCommand(header=TimestampHeader.construct_default())


def test_validator_rejects_both_variants_set(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        EETrajectoryCommand(
            header=random_timestamp_header(rng),
            ee_positions_trajectory=random_ee_positions_trajectory(rng),
            ee_velocities_trajectory=random_ee_velocities_trajectory(rng),
        )


def test_construct_default_passes_validator() -> None:
    cmd = EETrajectoryCommand.construct_default()
    assert cmd.ee_positions_trajectory is not None
    assert cmd.ee_velocities_trajectory is None


if __name__ == "__main__":
    run_manor_tests()
