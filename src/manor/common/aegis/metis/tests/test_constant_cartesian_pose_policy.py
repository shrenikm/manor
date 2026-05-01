"""
Tests for ConstantCartesianPosePolicy.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.aegis.metis.policies.constant_cartesian_pose_policy import (
    ConstantCartesianPosePolicy,
    ConstantCartesianPosePolicyConfig,
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


def _empty_observation() -> Observation:
    return Observation(header=TimestampHeader.from_system_time())


class TestProtocolCompliance:
    def test_satisfies_metis_policy_protocol(self) -> None:
        policy = ConstantCartesianPosePolicy.from_config(
            ConstantCartesianPosePolicyConfig(translation=np.array([0.2, 0.0, 0.3]))
        )
        assert isinstance(policy, MetisPolicy)

    def test_config_pins_enum(self) -> None:
        assert ConstantCartesianPosePolicyConfig.POLICY_TYPE is MetisPolicyType.CONSTANT_CARTESIAN_POSE


class TestActionShape:
    def test_emits_constant_cartesian_pose(self) -> None:
        translation = np.array([0.2, 0.05, 0.3], dtype=np.float64)
        orientation = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
        policy = ConstantCartesianPosePolicy.from_config(
            ConstantCartesianPosePolicyConfig(translation=translation, orientation=orientation)
        )
        action = policy.step(_empty_observation())
        assert action.cartesian_command is not None
        assert action.cartesian_command.cartesian_pose is not None
        np.testing.assert_array_equal(action.cartesian_command.cartesian_pose.translation, translation)
        np.testing.assert_array_equal(action.cartesian_command.cartesian_pose.orientation, orientation)

    def test_default_orientation_is_identity(self) -> None:
        translation = np.array([0.1, 0.0, 0.2], dtype=np.float64)
        policy = ConstantCartesianPosePolicy.from_config(ConstantCartesianPosePolicyConfig(translation=translation))
        action = policy.step(_empty_observation())
        np.testing.assert_array_equal(
            action.cartesian_command.cartesian_pose.orientation, np.array([1.0, 0.0, 0.0, 0.0])
        )


class TestPolicyManagerDispatch:
    def test_from_config(self) -> None:
        translation = np.array([0.2, 0.0, 0.3])
        policy = MetisPolicyManager.from_config(ConstantCartesianPosePolicyConfig(translation=translation))
        assert isinstance(policy, ConstantCartesianPosePolicy)

    def test_config_from_yaml(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict(
            {"type": "constant_cartesian_pose", "translation": [0.2, 0.0, 0.3]}
        )
        assert isinstance(config, ConstantCartesianPosePolicyConfig)
        np.testing.assert_array_equal(config.translation, [0.2, 0.0, 0.3])
        np.testing.assert_array_equal(config.orientation, [1.0, 0.0, 0.0, 0.0])

    def test_config_from_yaml_with_orientation(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict(
            {
                "type": "constant_cartesian_pose",
                "translation": [0.2, 0.0, 0.3],
                "orientation": [0.0, 1.0, 0.0, 0.0],
            }
        )
        np.testing.assert_array_equal(config.orientation, [0.0, 1.0, 0.0, 0.0])

    def test_config_from_yaml_rejects_bad_translation(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict({"type": "constant_cartesian_pose", "translation": [0.2, 0.0]})


if __name__ == "__main__":
    run_manor_tests()
