"""
Simulation ManipulatorBackend.

Closes over a ``Gaia`` instance: forwards Talos's outgoing Command
into the simulation, and reads the simulation's joint + EEF state
back. Gaia is *not* advanced from here -- that's the
``GaiaAdvancer`` LeafSystem's job.
"""

from __future__ import annotations

import attr
import numpy as np

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.definitions.command import Command
from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class SimManipulatorBackendConfig:
    """
    Configuration for the simulation manipulator backend. EEF DOF
    counts come from the manipulator model, not from this config.
    """


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

    def send_command(self, command: Command) -> None:
        if command.joint_positions is not None:
            self.gaia.apply_joint_position_command(command.joint_positions)
        elif command.joint_velocities is not None:
            self.gaia.apply_joint_velocity_command(command.joint_velocities)
        # EEF-pose / EEF-twist commands aren't yet wired into Gaia; they
        # fall through silently until a tracking controller is added.

    def read_joint_state(self) -> JointState:
        return self.gaia.read_joint_state()

    def read_eef_state(self) -> EEFState:
        # Gaia doesn't yet model gripper state separately; emit a zero
        # EEFState sized to the manipulator model's EEF DOF count.
        num_eef_dofs = self.gaia.manipulator_model.get_num_eef_dofs()
        header = TimestampHeader.from_system_time()
        return EEFState(
            header=header,
            eef_positions=EEFPositions(
                header=header,
                positions=np.zeros(num_eef_dofs, dtype=np.float64),
            ),
            eef_velocities=EEFVelocities(
                header=header,
                velocities=np.zeros(num_eef_dofs, dtype=np.float64),
            ),
        )
