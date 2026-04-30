"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK via an ``IManipulatorDriver``. The driver
is the source of truth for joint DOF / EE DOF counts; this backend just routes
``ManipulatorBackend`` calls (``send_command`` / ``read_joint_state`` /
``read_ee_state`` / ``start`` / ``stop``) to the corresponding driver
methods.
"""

from __future__ import annotations

from typing import Self

import attr
import numpy as np

from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.command import Command
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.manipulators.manipulator_driver import IManipulatorDriver


@attr.frozen
class HardwareManipulatorBackendConfig:
    """
    Hardware-specific knobs for the manipulator backend. joint DOF / EE DOF
    counts intentionally live on the driver, not here.
    """

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "hardware_backend_config"))


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
        joint_command = command.joint_command
        if joint_command.joint_positions is not None:
            self.driver.write_joint_positions(joint_command.joint_positions)
        elif joint_command.joint_velocities is not None:
            self.driver.write_joint_velocities(joint_command.joint_velocities)
        ee_command = command.ee_command
        if ee_command is not None:
            if ee_command.ee_positions is not None:
                self.driver.write_ee_positions(ee_command.ee_positions)
            elif ee_command.ee_velocities is not None:
                self.driver.write_ee_velocities(ee_command.ee_velocities)

    def read_joint_state(self) -> JointState:
        positions = self.driver.read_joint_positions()
        velocities = self.driver.read_joint_velocities()
        return JointState(
            header=TimestampHeader.from_system_time(),
            joint_positions=positions,
            joint_velocities=velocities,
        )

    def read_ee_state(self) -> EEState:
        positions = self.driver.read_ee_positions()
        velocities = self.driver.read_ee_velocities()
        header = TimestampHeader.from_system_time()
        if positions is None:
            positions = EEPositions(
                header=header,
                positions=np.zeros(self.driver.get_num_ee_dofs(), dtype=np.float64),
            )
        if velocities is None:
            velocities = EEVelocities(
                header=header,
                velocities=np.zeros(self.driver.get_num_ee_dofs(), dtype=np.float64),
            )
        return EEState(
            header=header,
            ee_positions=positions,
            ee_velocities=velocities,
        )
