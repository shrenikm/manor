"""
``PassthroughController``: forwards a joint-positions or
joint-velocities Action through as the matching Command variant.

Falls back to a zero-velocity command if the Action carries an
EEF-space variant (those need IK, which a passthrough can't provide).
Useful when a policy is already producing command-shaped actions and a
real low-level controller hasn't landed yet.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.kyber.controllers.controller_manager import (
    KyberControllerConfigBase,
    KyberControllerType,
)
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class PassthroughControllerConfig(KyberControllerConfigBase):
    """
    Config for ``PassthroughController``. ``num_dof`` sizes the
    fallback zero-velocity command emitted when an EEF-space Action
    arrives.
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType] = KyberControllerType.PASSTHROUGH

    num_dof: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "PassthroughControllerConfig"))


@attr.frozen
class PassthroughController:
    """
    Forward joint-space Actions through as the matching Command, with
    an EEF-space fallback to a zero-velocity command sized to
    ``num_dof``.
    """

    num_dof: int = 0

    def step(self, action: Action, proprioception: Proprioception) -> Command:
        del proprioception
        header = TimestampHeader.from_system_time()
        if action.joint_positions is not None:
            return Command(header=header, joint_positions=action.joint_positions)
        if action.joint_velocities is not None:
            return Command(header=header, joint_velocities=action.joint_velocities)
        return Command(
            header=header,
            joint_velocities=JointVelocities(
                header=header,
                velocities=np.zeros(self.num_dof, dtype=np.float64),
            ),
        )
