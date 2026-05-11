"""
ZeroVelocityController: emits a zero JointVelocities command, irrespective of action /
proprioception.

Default during early bring-up: lets the full aegis graph tick without moving the robot. num_dof sets
the command width.
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
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class ZeroVelocityControllerConfig(KyberControllerConfigBase):
    """
    Config for ZeroVelocityController. num_dof is required at runtime (constructable as 0 for tests,
    but a zero-width command will never be useful in a real diagram).
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType] = KyberControllerType.ZERO_VELOCITY

    num_dof: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "ZeroVelocityControllerConfig"))


@attr.frozen
class ZeroVelocityController:
    """
    Always emit a zero JointVelocities command sized to num_dof.
    """

    num_dof: int = 0

    def step(self, action: Action, proprioception: Proprioception) -> JointEECommand:
        del action, proprioception
        header = TimestampHeader.from_system_time()
        return JointEECommand(
            header=header,
            joint_command=JointCommand(
                header=header,
                joint_velocities=JointVelocities(
                    header=header,
                    velocities=np.zeros(self.num_dof, dtype=np.float64),
                ),
            ),
        )
