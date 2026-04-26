"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK via an ``IManipulatorDriver``. The driver
itself is responsible for prime / unprime + read / write -- the
backend's job is to fan those calls into Talos's ``ManipulatorBackend``
protocol shape (``send_command`` / ``read_joint_state`` /
``read_eef_state`` / ``start`` / ``stop``).
"""

from __future__ import annotations

import attr
import numpy as np

from manor.common.definitions.command import Command
from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.manipulators.manipulator_driver import IManipulatorDriver


@attr.frozen
class HardwareManipulatorBackendConfig:
    """
    Static configuration for the hardware manipulator backend.

    ``num_eef_dofs`` is the number of generalized DOFs reported on the
    EEFState message; the driver decides whether/how to populate them.
    """

    num_eef_dofs: int = 0


@attr.define
class HardwareManipulatorBackend:
    """
    ManipulatorBackend that delegates to an ``IManipulatorDriver``.
    """

    driver: IManipulatorDriver
    config: HardwareManipulatorBackendConfig = attr.field(factory=HardwareManipulatorBackendConfig)

    def start(self) -> None:
        self.driver.prime()

    def stop(self) -> None:
        self.driver.unprime()

    def send_command(self, command: Command) -> None:
        if command.joint_positions is not None:
            self.driver.write_joint_positions(command.joint_positions)
        elif command.joint_velocities is not None:
            self.driver.write_joint_velocities(command.joint_velocities)
        # EEF-pose / EEF-twist commands need IK before they reach the
        # driver; that path lives in Kyber, not here.

    def read_joint_state(self) -> JointState:
        positions = self.driver.read_joint_positions()
        velocities = self.driver.read_joint_velocities()
        return JointState(
            header=TimestampHeader.from_system_time(),
            joint_positions=positions,
            joint_velocities=velocities,
        )

    def read_eef_state(self) -> EEFState:
        positions = self.driver.read_eef_positions()
        velocities = self.driver.read_eef_velocities()
        header = TimestampHeader.from_system_time()
        if positions is None:
            positions = EEFPositions(
                header=header,
                positions=np.zeros(self.config.num_eef_dofs, dtype=np.float64),
            )
        if velocities is None:
            velocities = EEFVelocities(
                header=header,
                velocities=np.zeros(self.config.num_eef_dofs, dtype=np.float64),
            )
        return EEFState(
            header=header,
            eef_positions=positions,
            eef_velocities=velocities,
        )
