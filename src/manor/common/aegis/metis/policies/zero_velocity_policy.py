"""
``ZeroVelocityPolicy``: emits a zero ``JointVelocities`` action,
irrespective of the observation.

This is the default during early bring-up: Metis publishes, Kyber
subscribes, and the robot doesn't move. ``num_joints`` sets the action
width when the observation lacks proprioception; when proprioception
is present, the policy resizes itself off the incoming joint count.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import assert_keys_match_attrs, require_int
from manor.common.definitions.action import Action
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class ZeroVelocityPolicyConfig(MetisPolicyConfigBase):
    """
    Config for ``ZeroVelocityPolicy``. ``num_joints`` is only consulted
    when the observation lacks proprioception; with proprioception in
    hand, the policy reads the joint count off the incoming state.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.ZERO_VELOCITY

    num_joints: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        assert_keys_match_attrs(cls, d, "ZeroVelocityPolicyConfig")
        return cls(num_joints=require_int(d.get("num_joints", 0), "ZeroVelocityPolicyConfig.num_joints"))


@attr.frozen
class ZeroVelocityPolicy:
    """
    Always emit a zero ``JointVelocities`` action.

    ``num_joints`` sets the action's vector size; pulled from the
    observation when available so the action width matches the robot.
    """

    num_joints: int = 0

    def step(self, observation: Observation) -> Action:
        num_joints = self.num_joints
        if observation.proprioception is not None:
            num_joints = observation.proprioception.joint_state.joint_positions.positions.shape[0]
        header = TimestampHeader.from_system_time()
        return Action(
            header=header,
            joint_velocities=JointVelocities(
                header=header,
                velocities=np.zeros(num_joints, dtype=np.float64),
            ),
        )
