"""
Simulation ManipulatorBackend.

Closes over a ``Sim`` instance: forwards Talos's outgoing Command into
the simulation, and reads the simulation's joint + EEF state back. The
Sim is *not* advanced from here -- that's the ``_SimAdvancer``
LeafSystem's job.
"""

from __future__ import annotations

import attr
import numpy as np

from manor.common.aegis.sim.sim import Sim
from manor.common.definitions.command import Command
from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class SimManipulatorBackendConfig:
    """
    Configuration for the simulation manipulator backend.

    ``num_eef_dofs`` is the number of generalized DOFs reported on the
    EEFState message. The number of arm joints is implicit in the Sim's
    plant, not configured here.
    """

    num_eef_dofs: int = 0


@attr.define
class SimManipulatorBackend:
    """
    ManipulatorBackend that drives a shared ``Sim``.
    """

    sim: Sim
    config: SimManipulatorBackendConfig = attr.field(factory=SimManipulatorBackendConfig)

    def start(self) -> None:
        # Sim's lifecycle is managed by its owner; nothing to do per-run.
        return

    def stop(self) -> None:
        return

    def send_command(self, command: Command) -> None:
        if command.joint_positions is not None:
            self.sim.apply_joint_position_command(command.joint_positions.positions)
        elif command.joint_velocities is not None:
            self.sim.apply_joint_velocity_command(command.joint_velocities.velocities)
        # EEF-pose / EEF-twist commands aren't yet wired into the sim
        # backend; they fall through silently until a tracking
        # controller is added.

    def read_joint_state(self) -> JointState:
        return self.sim.read_joint_state()

    def read_eef_state(self) -> EEFState:
        # The Sim doesn't yet model gripper state separately; emit a
        # zero-DOF EEFState with a fresh timestamp.
        header = TimestampHeader.from_system_time()
        return EEFState(
            header=header,
            eef_positions=EEFPositions(
                header=header,
                positions=np.zeros(self.config.num_eef_dofs, dtype=np.float64),
            ),
            eef_velocities=EEFVelocities(
                header=header,
                velocities=np.zeros(self.config.num_eef_dofs, dtype=np.float64),
            ),
        )
