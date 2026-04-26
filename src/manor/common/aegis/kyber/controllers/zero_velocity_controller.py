"""
``ZeroVelocityController``: emits a zero ``JointVelocities`` command,
irrespective of action / proprioception.

Default during early bring-up: lets the full aegis graph tick without
moving the robot. ``num_dof`` sets the command width; aegis refuses
to build with ``num_dof <= 0`` since a zero-width command is never
useful.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.kyber.controllers.controller_manager import (
    KyberControllerConfigBase,
    KyberControllerType,
)
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError


class _YamlKey(StrEnum):
    NUM_DOF = "num_dof"


@attr.frozen
class ZeroVelocityControllerConfig(KyberControllerConfigBase):
    """
    Config for ``ZeroVelocityController``. ``num_dof`` is required at
    runtime (constructable as 0 for tests, but a zero-width command
    will never be useful in a real diagram).
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType] = KyberControllerType.ZERO_VELOCITY

    num_dof: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        allowed = {key.value for key in _YamlKey}
        extras = set(d) - allowed
        if extras:
            raise AegisConfigError(
                f"ZeroVelocityControllerConfig: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}"
            )
        num_dof_raw = d.get(_YamlKey.NUM_DOF, 0)
        if isinstance(num_dof_raw, bool) or not isinstance(num_dof_raw, int):
            raise AegisConfigError(
                f"ZeroVelocityControllerConfig.{_YamlKey.NUM_DOF} must be an int; got {type(num_dof_raw).__name__}"
            )
        return cls(num_dof=num_dof_raw)


@attr.frozen
class ZeroVelocityController:
    """
    Always emit a zero ``JointVelocities`` command sized to ``num_dof``.
    """

    num_dof: int = 0

    def step(self, action: Action, proprioception: Proprioception) -> Command:
        del action, proprioception
        header = TimestampHeader.from_system_time()
        return Command(
            header=header,
            joint_velocities=JointVelocities(
                header=header,
                velocities=np.zeros(self.num_dof, dtype=np.float64),
            ),
        )
