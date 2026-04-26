"""
Concrete Controller implementations for Kyber.

Two stub controllers are provided right now -- enough to exercise the
aegis graph end-to-end without a real low-level controller wired in:

* ``ZeroVelocityController``: emits a zero ``JointVelocities`` command
  irrespective of the action / proprioception. Default for the current
  high-level wiring phase.
* ``ActionPassthroughController``: forwards a joint-positions or
  joint-velocities Action through as the matching Command variant.
  Useful when a policy is already producing command-shaped actions
  and a real low-level controller hasn't landed yet.

Real controllers (diff-IK, joint-space tracking with PID, task-space
trajectory tracking) will land here as additional ``Controller``
subclasses.
"""

from __future__ import annotations

import attr
import numpy as np

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


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


@attr.frozen
class ActionPassthroughController:
    """
    Forward a joint-positions or joint-velocities Action through as the
    matching Command variant. Falls back to a zero-velocity command if
    the Action carries an EEF-space variant (those need IK, which a
    passthrough can't provide).
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
