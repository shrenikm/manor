"""
Round-trip tests for JointEECommand. Covers every combination of joint
command variant against either no EE command or each EE command variant.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.tests.factories import (
    EE_COMMAND_VARIANT_FIELDS,
    JOINT_COMMAND_VARIANT_FIELDS,
    random_joint_ee_command,
)
from manor.common.testing_utils import run_manor_tests

_EE_FIELD_OPTIONS: tuple[str | None, ...] = (None, *EE_COMMAND_VARIANT_FIELDS)


@pytest.mark.parametrize("joint_variant_field", JOINT_COMMAND_VARIANT_FIELDS)
@pytest.mark.parametrize("ee_variant_field", _EE_FIELD_OPTIONS)
def test_capnp_roundtrip(rng: np.random.Generator, joint_variant_field: str, ee_variant_field: str | None) -> None:
    original = random_joint_ee_command(rng, joint_variant_field, ee_variant_field)
    assert JointEECommand.deserialize(original.serialize()) == original


@pytest.mark.parametrize("joint_variant_field", JOINT_COMMAND_VARIANT_FIELDS)
@pytest.mark.parametrize("ee_variant_field", _EE_FIELD_OPTIONS)
def test_lcm_roundtrip(rng: np.random.Generator, joint_variant_field: str, ee_variant_field: str | None) -> None:
    original = random_joint_ee_command(rng, joint_variant_field, ee_variant_field)
    assert JointEECommand.from_lcm_message(original.to_lcm_message()) == original


if __name__ == "__main__":
    run_manor_tests()
