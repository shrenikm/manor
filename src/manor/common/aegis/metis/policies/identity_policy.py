"""
``IdentityPolicy``: mirrors the current measured joint positions back
out as a joint-positions action. Useful as a sanity check when closing
the loop on a position-tracking controller.

When the observation lacks proprioception, the policy emits a zeroed
``JointPositions`` of width ``num_joints``.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import assert_keys_match_attrs, require_int
from manor.common.definitions.action import Action
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class IdentityPolicyConfig(MetisPolicyConfigBase):
    """
    Config for ``IdentityPolicy``. ``num_joints`` sets the fallback
    action width when the observation lacks proprioception.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.IDENTITY

    num_joints: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        assert_keys_match_attrs(cls, d, "IdentityPolicyConfig")
        return cls(num_joints=require_int(d.get("num_joints", 0), "IdentityPolicyConfig.num_joints"))


@attr.frozen
class IdentityPolicy:
    """
    Mirrors the current joint positions back out as a joint-positions Action.
    """

    num_joints: int = 0

    def step(self, observation: Observation) -> Action:
        if observation.proprioception is not None:
            joint_positions = observation.proprioception.joint_state.joint_positions
            joint_positions = attr.evolve(joint_positions, header=TimestampHeader.from_system_time())
        else:
            joint_positions = JointPositions.construct_default(num_joints=self.num_joints)

        return Action(header=TimestampHeader.from_system_time(), joint_positions=joint_positions)
