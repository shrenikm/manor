"""
``ZeroVelocityPolicy``: emits a zero ``JointVelocities`` action,
irrespective of the observation.

This is the default during early bring-up: Metis publishes, Kyber
subscribes, and the robot doesn't move. ``num_joints`` sets the action
width when the observation lacks proprioception; when proprioception
is present, the policy resizes itself off the incoming joint count.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.definitions.action import Action
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError


class _YamlKey(StrEnum):
    NUM_JOINTS = "num_joints"


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
        allowed = {key.value for key in _YamlKey}
        extras = set(d) - allowed
        if extras:
            raise AegisConfigError(
                f"ZeroVelocityPolicyConfig: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}"
            )
        num_joints_raw = d.get(_YamlKey.NUM_JOINTS, 0)
        if isinstance(num_joints_raw, bool) or not isinstance(num_joints_raw, int):
            raise AegisConfigError(
                f"ZeroVelocityPolicyConfig.{_YamlKey.NUM_JOINTS} must be an int; got {type(num_joints_raw).__name__}"
            )
        return cls(num_joints=num_joints_raw)


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
