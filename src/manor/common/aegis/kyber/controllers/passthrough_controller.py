"""
PassthroughController: forwards an Action's group-1 joint command and
group-2 ee command straight through as the matching JointEECommand.
The gripper side passes through verbatim; the arm side passes only
when the Action carries an instantaneous JointCommand.

Cartesian or trajectory arm shapes need IK or trajectory tracking, which
a passthrough cannot provide, so the controller falls back to a
zero-velocity joint command sized to num_dof in those cases.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr

from manor.common.aegis.kyber.controllers.controller_manager import (
    KyberControllerConfigBase,
    KyberControllerType,
)
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.action import Action
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class PassthroughControllerConfig(KyberControllerConfigBase):
    """
    Config for PassthroughController. num_dof sizes the fallback
    zero-velocity joint command emitted when the Action's arm side is
    Cartesian or trajectory-shaped.
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType] = KyberControllerType.PASSTHROUGH

    num_dof: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "PassthroughControllerConfig"))


@attr.frozen
class PassthroughController:
    """
    Forward an Action's joint and ee command sides straight through as
    the matching JointEECommand. Falls back to a zero-velocity joint
    command sized to num_dof when the Action's arm side is Cartesian or
    trajectory-shaped.
    """

    num_dof: int = 0

    def step(self, action: Action, proprioception: Proprioception) -> JointEECommand:
        del proprioception
        header = TimestampHeader.from_system_time()
        if action.joint_command is not None:
            joint_command = action.joint_command
        else:
            raise NotImplementedError("PassthroughController only supports joint commands.")
        return JointEECommand(header=header, joint_command=joint_command, ee_command=action.ee_command)
