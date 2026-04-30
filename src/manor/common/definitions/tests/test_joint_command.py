"""
Round-trip + validator tests for JointCommand.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.tests.factories import (
    JOINT_COMMAND_VARIANT_FIELDS,
    random_joint_command,
    random_joint_positions,
    random_joint_velocities,
    random_timestamp_header,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError


@pytest.mark.parametrize("variant_field", JOINT_COMMAND_VARIANT_FIELDS)
def test_capnp_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_joint_command(rng, variant_field)
    assert JointCommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("variant_field", JOINT_COMMAND_VARIANT_FIELDS)
def test_lcm_roundtrip(rng: np.random.Generator, variant_field: str) -> None:
    original = random_joint_command(rng, variant_field)
    assert JointCommand.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_variant_set() -> None:
    with pytest.raises(InvalidDefinitionError):
        JointCommand(header=TimestampHeader.construct_default())


def test_validator_rejects_both_variants_set(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        JointCommand(
            header=random_timestamp_header(rng),
            joint_positions=random_joint_positions(rng),
            joint_velocities=random_joint_velocities(rng),
        )


def test_construct_default_passes_validator() -> None:
    cmd = JointCommand.construct_default()
    assert cmd.joint_positions is not None
    assert cmd.joint_velocities is None


def test_capnp_arm_dispatch(rng: np.random.Generator) -> None:
    cmd = JointCommand(header=random_timestamp_header(rng), joint_velocities=random_joint_velocities(rng))
    serialized = cmd.serialize()
    restored = JointCommand.deserialize(serialized)
    assert restored.joint_positions is None
    assert restored.joint_velocities is not None
    assert isinstance(restored.joint_velocities, JointVelocities)


def test_lcm_variant_tag_dispatch(rng: np.random.Generator) -> None:
    cmd = JointCommand(header=random_timestamp_header(rng), joint_positions=random_joint_positions(rng))
    msg = cmd.to_lcm_message()
    assert msg.variant == 0  # joint_positions is the first variant field
    restored = JointCommand.from_lcm_message(msg)
    assert restored.joint_positions is not None
    assert isinstance(restored.joint_positions, JointPositions)
    assert restored.joint_velocities is None
