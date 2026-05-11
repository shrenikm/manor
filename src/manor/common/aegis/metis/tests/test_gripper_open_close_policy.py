"""
Tests for GripperOpenClosePolicy.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from manor.common.aegis.metis.policies.gripper_open_close_policy import (
    GripperOpenClosePolicy,
    GripperOpenClosePolicyConfig,
)
from manor.common.aegis.metis.policies.policy_manager import (
    MetisPolicy,
    MetisPolicyManager,
    MetisPolicyType,
)
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import (
    LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
    LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M,
    Lite6Model,
)
from manor.manipulators.lite6.variant import Lite6Variant


def _empty_observation() -> Observation:
    return Observation(header=TimestampHeader.from_system_time())


def _np_model() -> Lite6Model:
    return Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL)


class TestProtocolCompliance:
    def test_satisfies_metis_policy_protocol(self) -> None:
        policy = GripperOpenClosePolicy.from_config(
            config=GripperOpenClosePolicyConfig(num_arm_dof=6, open_hold_seconds=0.1, close_hold_seconds=0.1),
            manipulator_model=_np_model(),
        )
        assert isinstance(policy, MetisPolicy)

    def test_config_pins_enum(self) -> None:
        assert GripperOpenClosePolicyConfig.POLICY_TYPE is MetisPolicyType.GRIPPER_OPEN_CLOSE


class TestActionShape:
    def test_emits_zero_velocity_arm_and_ee_positions(self) -> None:
        policy = GripperOpenClosePolicy.from_config(
            config=GripperOpenClosePolicyConfig(
                num_arm_dof=6,
                open_hold_seconds=0.05,
                close_hold_seconds=0.05,
                num_cycles=1,
            ),
            manipulator_model=_np_model(),
        )
        action = policy.step(_empty_observation())
        assert action.joint_command.joint_velocities is not None
        np.testing.assert_array_equal(action.joint_command.joint_velocities.velocities, np.zeros(6, dtype=np.float64))
        assert action.ee_command is not None
        assert action.ee_command.ee_positions is not None
        assert action.ee_command.ee_positions.positions.shape == (1,)


class TestCycleBehaviour:
    def test_starts_with_open_then_transitions_to_close(self) -> None:
        # open_hold = 0.05s, close_hold = 0.05s, 1 cycle. The first
        # step latches t0; after open_hold the policy should switch to
        # close. Open / close widths come from the model.
        policy = GripperOpenClosePolicy.from_config(
            config=GripperOpenClosePolicyConfig(
                num_arm_dof=6,
                open_hold_seconds=0.05,
                close_hold_seconds=0.05,
                num_cycles=1,
            ),
            manipulator_model=_np_model(),
        )
        first = policy.step(_empty_observation())
        np.testing.assert_allclose(first.ee_command.ee_positions.positions, [LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M])
        time.sleep(0.07)
        second = policy.step(_empty_observation())
        np.testing.assert_allclose(second.ee_command.ee_positions.positions, [LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M])

    def test_settles_at_close_after_all_cycles(self) -> None:
        policy = GripperOpenClosePolicy.from_config(
            config=GripperOpenClosePolicyConfig(
                num_arm_dof=6,
                open_hold_seconds=0.02,
                close_hold_seconds=0.02,
                num_cycles=1,
            ),
            manipulator_model=_np_model(),
        )
        policy.step(_empty_observation())
        time.sleep(0.06)
        action = policy.step(_empty_observation())
        np.testing.assert_allclose(action.ee_command.ee_positions.positions, [LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M])


class TestPolicyManagerDispatch:
    def test_from_config(self) -> None:
        policy = MetisPolicyManager.from_config(
            GripperOpenClosePolicyConfig(num_arm_dof=6),
            manipulator_model=_np_model(),
        )
        assert isinstance(policy, GripperOpenClosePolicy)

    def test_from_config_without_model_raises(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.from_config(GripperOpenClosePolicyConfig(num_arm_dof=6))

    def test_config_from_yaml(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict(
            {"type": "gripper_open_close", "num_arm_dof": 6, "open_hold_seconds": 0.5}
        )
        assert isinstance(config, GripperOpenClosePolicyConfig)
        assert config.num_arm_dof == 6
        assert config.open_hold_seconds == 0.5


if __name__ == "__main__":
    run_manor_tests()
