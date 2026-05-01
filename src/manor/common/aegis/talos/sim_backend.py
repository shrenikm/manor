"""
Simulation ManipulatorBackend.

Closes over a ``Gaia`` instance: forwards Talos's outgoing
JointEECommand into the simulation, and reads the simulation's
joint + EE state back. Gaia is *not* advanced from here -- that's
the ``GaiaAdvancer`` LeafSystem's job.
"""

from __future__ import annotations

from typing import Self

import attr
import numpy as np

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class SimManipulatorBackendConfig:
    """
    Configuration for the simulation manipulator backend. EE DOF
    counts come from the manipulator model, not from this config.
    """

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "sim_backend_config"))


@attr.define
class SimManipulatorBackend:
    """
    ManipulatorBackend that drives a shared ``Gaia``.
    """

    gaia: Gaia
    config: SimManipulatorBackendConfig = attr.field(factory=SimManipulatorBackendConfig)

    def start(self) -> None:
        # Gaia's lifecycle is managed by its owner; nothing to do per-run.
        return

    def stop(self) -> None:
        return

    def send_joint_ee_command(self, joint_ee_command: JointEECommand) -> None:
        joint_command = joint_ee_command.joint_command
        if joint_command.joint_positions is not None:
            self.gaia.apply_joint_position_command(joint_command.joint_positions)
        elif joint_command.joint_velocities is not None:
            self.gaia.apply_joint_velocity_command(joint_command.joint_velocities)
        # EE-side commands aren't wired into Gaia yet; gripper joints
        # are held at their measured pose by Gaia's _DesiredStateSource,
        # so joint_ee_command.ee_command falls through silently until a
        # gripper tracking path lands.

    def read_joint_state(self) -> JointState:
        return self.gaia.read_joint_state()

    def read_ee_state(self) -> EEState:
        # Gaia doesn't yet model gripper state separately; emit a zero
        # EEState sized to the manipulator model's EE DOF count.
        num_ee_dofs = self.gaia.manipulator_model.get_num_ee_dofs()
        header = TimestampHeader.from_system_time()
        return EEState(
            header=header,
            ee_positions=EEPositions(
                header=header,
                positions=np.zeros(num_ee_dofs, dtype=np.float64),
            ),
            ee_velocities=EEVelocities(
                header=header,
                velocities=np.zeros(num_ee_dofs, dtype=np.float64),
            ),
        )
