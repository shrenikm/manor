"""
Tests for Metis: port shape, policy invocation, identity policy, and YAML round-tripping for
MetisConfig.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies.constant_policies import (
    ConstantJointPositionsPolicy,
    ConstantJointPositionsPolicyConfig,
    ConstantJointVelocitiesPolicy,
    ConstantJointVelocitiesPolicyConfig,
)
from manor.common.aegis.metis.policies.identity_policy import IdentityPolicy, IdentityPolicyConfig
from manor.common.aegis.metis.policies.policy_manager import (
    MetisPolicy,
    MetisPolicyManager,
    MetisPolicyType,
)
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError
from manor.common.testing_utils import run_manor_tests


def _make_proprioception(positions: np.ndarray) -> Proprioception:
    n = positions.size
    return Proprioception(
        header=TimestampHeader(monotonic_ns=1, system_ns=2),
        joint_state=JointState(
            header=TimestampHeader(monotonic_ns=3, system_ns=4),
            joint_positions=JointPositions(
                header=TimestampHeader(monotonic_ns=5, system_ns=6),
                positions=positions,
            ),
            joint_velocities=JointVelocities(
                header=TimestampHeader(monotonic_ns=7, system_ns=8),
                velocities=np.zeros(n, dtype=np.float64),
            ),
        ),
    )


class _RecordingPolicy:
    def __init__(self, canned_action: Action) -> None:
        self.canned = canned_action
        self.observations: list[Observation] = []

    def step(self, observation: Observation) -> Action:
        self.observations.append(observation)
        return self.canned


class TestMetisConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            Metis(policy=IdentityPolicy(), publish_frequency=0.0)

    def test_declares_expected_ports(self) -> None:
        metis = Metis(policy=IdentityPolicy(), publish_frequency=10.0)
        assert metis.num_input_ports() == 3
        assert metis.num_output_ports() == 1
        assert metis.GetInputPort(MetisPorts.INPUT_PROPRIOCEPTION) is not None
        assert metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE) is not None
        assert metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE) is not None
        assert metis.GetOutputPort(MetisPorts.OUTPUT_ACTION) is not None


class TestMetisPolicyFlow:
    def test_policy_receives_observation_and_action_is_published(self) -> None:
        positions = np.array([0.1, 0.2, 0.3], dtype=np.float64)
        canned = Action(
            header=TimestampHeader(monotonic_ns=100, system_ns=101),
            joint_command=JointCommand(
                header=TimestampHeader(monotonic_ns=102, system_ns=103),
                joint_positions=JointPositions(
                    header=TimestampHeader(monotonic_ns=104, system_ns=105),
                    positions=positions,
                ),
            ),
        )
        policy = _RecordingPolicy(canned)
        metis = Metis(policy=policy, publish_frequency=100.0)
        context = metis.CreateDefaultContext()
        metis.GetInputPort(MetisPorts.INPUT_PROPRIOCEPTION).FixValue(
            context, AbstractValue.Make(_make_proprioception(positions))
        )
        metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE).FixValue(
            context, AbstractValue.Make(RGBImageData.construct_default())
        )
        metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE).FixValue(
            context, AbstractValue.Make(DepthImageData.construct_default())
        )

        simulator = Simulator(metis, context)
        simulator.AdvanceTo(0.05)

        assert len(policy.observations) >= 1
        action = metis.GetOutputPort(MetisPorts.OUTPUT_ACTION).Eval(simulator.get_context())
        np.testing.assert_array_equal(action.joint_command.joint_positions.positions, positions)


class TestIdentityPolicy:
    def test_is_a_policy(self) -> None:
        assert isinstance(IdentityPolicy(), MetisPolicy)

    def test_mirrors_current_joint_positions(self) -> None:
        policy = IdentityPolicy()
        positions = np.array([0.5, -0.5, 1.0], dtype=np.float64)
        observation = Observation(
            header=TimestampHeader(monotonic_ns=1, system_ns=2),
            proprioception=_make_proprioception(positions),
        )
        action = policy.step(observation)
        np.testing.assert_array_equal(action.joint_command.joint_positions.positions, positions)

    def test_falls_back_to_defaults_without_proprioception(self) -> None:
        policy = IdentityPolicy(num_joints=4)
        observation = Observation(header=TimestampHeader(monotonic_ns=1, system_ns=2))
        action = policy.step(observation)
        assert action.joint_command is not None
        assert action.joint_command.joint_positions is not None
        assert action.joint_command.joint_positions.positions.shape == (4,)


class TestConstantJointPositionsPolicy:
    def test_is_a_policy(self) -> None:
        assert isinstance(ConstantJointPositionsPolicy(positions=np.zeros(6)), MetisPolicy)

    def test_emits_constant_positions_irrespective_of_observation(self) -> None:
        target = np.array([0.1, -0.2, 0.3, 0.0, 0.5, -0.4], dtype=np.float64)
        policy = ConstantJointPositionsPolicy(positions=target)

        observed = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float64)
        observation_with = Observation(
            header=TimestampHeader(monotonic_ns=1, system_ns=2),
            proprioception=_make_proprioception(observed),
        )
        observation_without = Observation(header=TimestampHeader(monotonic_ns=3, system_ns=4))

        for observation in (observation_with, observation_without):
            action = policy.step(observation)
            assert action.joint_command is not None
            assert action.joint_command.joint_positions is not None
            np.testing.assert_array_equal(action.joint_command.joint_positions.positions, target)

    def test_action_does_not_alias_config_array(self) -> None:
        target = np.array([0.1, 0.2, 0.3], dtype=np.float64)
        policy = ConstantJointPositionsPolicy(positions=target)
        action = policy.step(Observation(header=TimestampHeader(monotonic_ns=1, system_ns=2)))
        assert action.joint_command is not None
        assert action.joint_command.joint_positions is not None
        action.joint_command.joint_positions.positions[0] = 99.0
        assert policy.positions[0] == 0.1


class TestConstantJointVelocitiesPolicy:
    def test_is_a_policy(self) -> None:
        assert isinstance(ConstantJointVelocitiesPolicy(velocities=np.zeros(6)), MetisPolicy)

    def test_emits_constant_velocities_irrespective_of_observation(self) -> None:
        target = np.array([0.05, -0.1, 0.2, 0.0, -0.3, 0.15], dtype=np.float64)
        policy = ConstantJointVelocitiesPolicy(velocities=target)

        observed = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float64)
        observation_with = Observation(
            header=TimestampHeader(monotonic_ns=1, system_ns=2),
            proprioception=_make_proprioception(observed),
        )
        observation_without = Observation(header=TimestampHeader(monotonic_ns=3, system_ns=4))

        for observation in (observation_with, observation_without):
            action = policy.step(observation)
            assert action.joint_command is not None
            assert action.joint_command.joint_velocities is not None
            np.testing.assert_array_equal(action.joint_command.joint_velocities.velocities, target)

    def test_zero_velocities_special_case(self) -> None:
        policy = ConstantJointVelocitiesPolicy(velocities=np.zeros(6, dtype=np.float64))
        action = policy.step(Observation(header=TimestampHeader(monotonic_ns=1, system_ns=2)))
        assert action.joint_command is not None
        assert action.joint_command.joint_velocities is not None
        np.testing.assert_array_equal(action.joint_command.joint_velocities.velocities, np.zeros(6))

    def test_action_does_not_alias_config_array(self) -> None:
        target = np.array([0.1, 0.2, 0.3], dtype=np.float64)
        policy = ConstantJointVelocitiesPolicy(velocities=target)
        action = policy.step(Observation(header=TimestampHeader(monotonic_ns=1, system_ns=2)))
        assert action.joint_command is not None
        assert action.joint_command.joint_velocities is not None
        action.joint_command.joint_velocities.velocities[0] = 99.0
        assert policy.velocities[0] == 0.1


class TestMetisPolicyConfigs:
    def test_identity_config_pins_enum(self) -> None:
        assert IdentityPolicyConfig.POLICY_TYPE is MetisPolicyType.IDENTITY

    def test_constant_joint_positions_config_pins_enum(self) -> None:
        assert ConstantJointPositionsPolicyConfig.POLICY_TYPE is MetisPolicyType.CONSTANT_JOINT_POSITIONS

    def test_constant_joint_velocities_config_pins_enum(self) -> None:
        assert ConstantJointVelocitiesPolicyConfig.POLICY_TYPE is MetisPolicyType.CONSTANT_JOINT_VELOCITIES


class TestMetisPolicyManager:
    def test_from_config_identity(self) -> None:
        policy = MetisPolicyManager.from_config(IdentityPolicyConfig(num_joints=3))
        assert isinstance(policy, IdentityPolicy)
        assert policy.num_joints == 3

    def test_from_config_constant_joint_positions(self) -> None:
        target = np.array([0.0, 0.5, -0.5, 1.0, 0.1, 0.2], dtype=np.float64)
        policy = MetisPolicyManager.from_config(ConstantJointPositionsPolicyConfig(positions=target))
        assert isinstance(policy, ConstantJointPositionsPolicy)
        np.testing.assert_array_equal(policy.positions, target)

    def test_from_config_constant_joint_velocities(self) -> None:
        target = np.array([0.0, 0.1, -0.1, 0.05, -0.05, 0.0], dtype=np.float64)
        policy = MetisPolicyManager.from_config(ConstantJointVelocitiesPolicyConfig(velocities=target))
        assert isinstance(policy, ConstantJointVelocitiesPolicy)
        np.testing.assert_array_equal(policy.velocities, target)

    def test_config_from_yaml_dict_identity(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict({"type": "identity", "num_joints": 4})
        assert isinstance(config, IdentityPolicyConfig)
        assert config.num_joints == 4

    def test_config_from_yaml_dict_constant_joint_positions(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict(
            {"type": "constant_joint_positions", "positions": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]}
        )
        assert isinstance(config, ConstantJointPositionsPolicyConfig)
        np.testing.assert_array_equal(config.positions, np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5], dtype=np.float64))

    def test_config_from_yaml_dict_constant_joint_positions_rejects_empty_list(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict({"type": "constant_joint_positions", "positions": []})

    def test_config_from_yaml_dict_constant_joint_positions_rejects_non_numeric_entry(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict(
                {"type": "constant_joint_positions", "positions": [0.0, "oops", 0.2]}
            )

    def test_config_from_yaml_dict_constant_joint_velocities(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict(
            {"type": "constant_joint_velocities", "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        )
        assert isinstance(config, ConstantJointVelocitiesPolicyConfig)
        np.testing.assert_array_equal(config.velocities, np.zeros(6, dtype=np.float64))

    def test_config_from_yaml_dict_constant_joint_velocities_rejects_empty_list(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict({"type": "constant_joint_velocities", "velocities": []})

    def test_config_from_yaml_dict_rejects_missing_type(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict({"num_joints": 6})

    def test_config_from_yaml_dict_rejects_unknown_type(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict({"type": "made_up", "num_joints": 6})

    def test_config_from_yaml_dict_rejects_unknown_keys(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisPolicyManager.config_from_yaml_dict({"type": "identity", "garbage": 7})


class TestMetisConfigYaml:
    def test_round_trips_minimal_block(self) -> None:
        config = MetisConfig.from_yaml_dict(
            {
                "policy_config": {"type": "constant_joint_velocities", "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            }
        )
        assert isinstance(config.policy_config, ConstantJointVelocitiesPolicyConfig)
        assert config.publish_frequency_hz == 10.0

    def test_round_trips_full_block(self) -> None:
        config = MetisConfig.from_yaml_dict(
            {
                "publish_frequency_hz": 25.0,
                "policy_config": {"type": "identity", "num_joints": 7},
            }
        )
        assert config.publish_frequency_hz == 25.0
        assert isinstance(config.policy_config, IdentityPolicyConfig)
        assert config.policy_config.num_joints == 7

    def test_rejects_missing_policy_config(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisConfig.from_yaml_dict({"publish_frequency_hz": 10.0})

    def test_rejects_unknown_keys(self) -> None:
        with pytest.raises(AegisConfigError):
            MetisConfig.from_yaml_dict(
                {
                    "policy_config": {"type": "identity", "num_joints": 6},
                    "extra": True,
                }
            )


if __name__ == "__main__":
    run_manor_tests()
