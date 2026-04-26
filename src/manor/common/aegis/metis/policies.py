"""
Concrete Policy implementations for Metis.

Two stub policies are provided right now -- enough to exercise the
aegis graph end-to-end without wiring in a real planner / learned
model:

* ``ZeroVelocityPolicy``: emits a zero ``JointVelocities`` action,
  irrespective of the observation. This is the default for the current
  high-level wiring phase: Metis publishes, Kyber subscribes, Talos
  receives a zero-velocity command and the robot doesn't move.
* ``IdentityPolicy``: mirrors the current measured joint positions back
  out as a joint-positions action. Useful as a sanity check when
  closing the loop on a position-tracking controller.

Real policies (motion planning, trajopt, diffusion, VLAs) will land
here as additional ``Policy`` subclasses.
"""

from __future__ import annotations

import attr
import numpy as np

from manor.common.definitions.action import Action
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader


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
