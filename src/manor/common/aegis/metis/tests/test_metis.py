"""
Tests for Metis: port shape, policy invocation, identity policy.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.metis.metis import Metis, MetisPorts, Policy
from manor.common.aegis.metis.policies import IdentityPolicy
from manor.common.definitions.action import Action
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.defaults import construct_default_depth_image, construct_default_rgb_image
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
            joint_positions=JointPositions(
                header=TimestampHeader(monotonic_ns=102, system_ns=103),
                positions=positions,
            ),
        )
        policy = _RecordingPolicy(canned)
        metis = Metis(policy=policy, publish_frequency=100.0)
        context = metis.CreateDefaultContext()
        metis.GetInputPort(MetisPorts.INPUT_PROPRIOCEPTION).FixValue(
            context, AbstractValue.Make(_make_proprioception(positions))
        )
        metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE).FixValue(
            context, AbstractValue.Make(construct_default_rgb_image())
        )
        metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE).FixValue(
            context, AbstractValue.Make(construct_default_depth_image())
        )

        simulator = Simulator(metis, context)
        simulator.AdvanceTo(0.05)

        assert len(policy.observations) >= 1
        action = metis.GetOutputPort(MetisPorts.OUTPUT_ACTION).Eval(simulator.get_context())
        np.testing.assert_array_equal(action.joint_positions.positions, positions)


class TestIdentityPolicy:
    def test_is_a_policy(self) -> None:
        assert isinstance(IdentityPolicy(), Policy)

    def test_mirrors_current_joint_positions(self) -> None:
        policy = IdentityPolicy()
        positions = np.array([0.5, -0.5, 1.0], dtype=np.float64)
        observation = Observation(
            header=TimestampHeader(monotonic_ns=1, system_ns=2),
            proprioception=_make_proprioception(positions),
        )
        action = policy.step(observation)
        np.testing.assert_array_equal(action.joint_positions.positions, positions)

    def test_falls_back_to_defaults_without_proprioception(self) -> None:
        policy = IdentityPolicy(num_joints=4)
        observation = Observation(header=TimestampHeader(monotonic_ns=1, system_ns=2))
        action = policy.step(observation)
        assert action.joint_positions is not None
        assert action.joint_positions.positions.shape == (4,)


if __name__ == "__main__":
    run_manor_tests()
