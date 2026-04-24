"""
Simulation ManipulatorBackend.

Drives a Drake MultibodyPlant: applies the latest commanded joint positions
to the plant's actuation input and reads plant state back. The concrete
plant wiring is deferred; the stub stores the last command and echoes its
joint positions back as the "measured" state so the Aegis graph can be
built, ticked, and observed end-to-end.
"""

from __future__ import annotations

from typing import Any

import attr

from manor.common.definitions.command import Command
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.utils.defaults import (
    construct_default_eef_state,
    construct_default_joint_positions,
    construct_default_joint_velocities,
    construct_system_time_header,
)


@attr.frozen
class SimManipulatorBackendConfig:
    """
    Configuration for the simulation manipulator backend.
    """

    num_joints: int = 6
    num_eef_dofs: int = 1


@attr.define
class SimManipulatorBackend:
    """
    ManipulatorBackend that drives a Drake plant.

    The ``plant`` / ``plant_context`` handles will carry a real
    MultibodyPlant and its Context once wired; for now they are accepted
    but unused by the stub.
    """

    config: SimManipulatorBackendConfig = attr.field(factory=SimManipulatorBackendConfig)
    plant: Any = None
    plant_context: Any = None
    _latest_command: Command | None = attr.field(default=None, init=False)

    def start(self) -> None:
        # TODO: reset plant state, zero actuation, go to a home configuration.
        return

    def stop(self) -> None:
        # TODO: leave the plant in a safe state.
        return

    def send_command(self, command: Command) -> None:
        # TODO: translate command into plant actuation input and advance the plant context.
        self._latest_command = command

    def read_joint_state(self) -> JointState:
        # TODO: read actual positions / velocities from the plant context.
        positions = (
            self._latest_command.joint_positions.positions
            if self._latest_command is not None and self._latest_command.joint_positions is not None
            else construct_default_joint_positions(self.config.num_joints).positions
        )
        return JointState(
            header=construct_system_time_header(),
            joint_positions=attr.evolve(
                construct_default_joint_positions(self.config.num_joints),
                header=construct_system_time_header(),
                positions=positions,
            ),
            joint_velocities=attr.evolve(
                construct_default_joint_velocities(self.config.num_joints),
                header=construct_system_time_header(),
            ),
        )

    def read_eef_state(self) -> EEFState:
        # TODO: derive from plant state or from a parallel-gripper model.
        return attr.evolve(construct_default_eef_state(self.config.num_eef_dofs), header=construct_system_time_header())
