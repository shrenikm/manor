"""
Round-trip and validator tests for Action. Covers every combination of an
arm command (group 1, exactly-one) against either no ee command or each
ee command variant (group 2, at-most-one), plus invalid configurations.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_trajectory_command import EETrajectoryCommand
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.tests.factories import (
    ACTION_ARM_FIELDS,
    ACTION_EE_FIELDS,
    random_action,
    random_cartesian_command,
    random_ee_command,
    random_ee_trajectory_command,
    random_joint_command,
    random_timestamp_header,
)
from manor.common.exceptions import InvalidDefinitionError
from manor.common.testing_utils import run_manor_tests

_EE_FIELD_OPTIONS: tuple[str | None, ...] = (None, *ACTION_EE_FIELDS)


@pytest.mark.parametrize("arm_field", ACTION_ARM_FIELDS)
@pytest.mark.parametrize("ee_field", _EE_FIELD_OPTIONS)
def test_capnp_roundtrip(rng: np.random.Generator, arm_field: str, ee_field: str | None) -> None:
    original = random_action(rng, arm_field, ee_field)
    assert Action.deserialize(original.serialize()) == original


@pytest.mark.parametrize("arm_field", ACTION_ARM_FIELDS)
@pytest.mark.parametrize("ee_field", _EE_FIELD_OPTIONS)
def test_lcm_roundtrip(rng: np.random.Generator, arm_field: str, ee_field: str | None) -> None:
    original = random_action(rng, arm_field, ee_field)
    assert Action.from_lcm_message(original.to_lcm_message()) == original


def test_validator_rejects_no_arm_command(rng: np.random.Generator) -> None:
    with pytest.raises(InvalidDefinitionError):
        Action(header=random_timestamp_header(rng))


def test_validator_rejects_two_arm_commands(rng: np.random.Generator) -> None:
    joint_cmd: JointCommand = random_joint_command(rng, "joint_positions")
    cartesian_cmd: CartesianCommand = random_cartesian_command(rng, "cartesian_pose")
    with pytest.raises(InvalidDefinitionError):
        Action(
            header=random_timestamp_header(rng),
            joint_command=joint_cmd,
            cartesian_command=cartesian_cmd,
        )


def test_validator_rejects_two_ee_commands(rng: np.random.Generator) -> None:
    joint_cmd: JointCommand = random_joint_command(rng, "joint_positions")
    ee_cmd: EECommand = random_ee_command(rng, "ee_positions")
    ee_traj_cmd: EETrajectoryCommand = random_ee_trajectory_command(rng, "ee_positions_trajectory")
    with pytest.raises(InvalidDefinitionError):
        Action(
            header=random_timestamp_header(rng),
            joint_command=joint_cmd,
            ee_command=ee_cmd,
            ee_trajectory_command=ee_traj_cmd,
        )


if __name__ == "__main__":
    run_manor_tests()
