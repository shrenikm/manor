"""
Concrete Policy implementations for Metis.

The initial set is a single identity-style passthrough that lets the Aegis
graph be exercised end-to-end with no real learning / planning wired in.
Classical motion planning, trajectory optimization, diffusion policies, and
VLAs will land here as independent Policy subclasses.
"""

from __future__ import annotations

import attr

from manor.common.definitions.action import Action
from manor.common.definitions.observation import Observation
from manor.common.definitions.utils.defaults import construct_default_joint_positions, construct_system_time_header


@attr.frozen
class IdentityPolicy:
    """
    Mirrors the current joint positions back out as a joint-positions Action.

    Useful only as a placeholder: running this policy produces a robot that
    tries to hold its current configuration. It validates the observation
    schema and gives Metis something non-trivial to publish before real
    policies are plugged in.
    """

    num_joints: int = 0

    def step(self, observation: Observation) -> Action:
        if observation.proprioception is not None:
            joint_positions = observation.proprioception.joint_state.joint_positions
            joint_positions = attr.evolve(joint_positions, header=construct_system_time_header())
        else:
            joint_positions = construct_default_joint_positions(self.num_joints)

        return Action(header=construct_system_time_header(), joint_positions=joint_positions)
