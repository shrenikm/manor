"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK via an ``IManipulatorDriver``. The driver
is the source of truth for DOF / EEF counts; this backend just routes
``ManipulatorBackend`` calls (``send_command`` / ``read_joint_state`` /
``read_eef_state`` / ``start`` / ``stop``) to the corresponding driver
methods.
"""

from __future__ import annotations

from typing import Self

import attr
import numpy as np

from manor.common.aegis.yaml_utils import parse_attrs_yaml
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
    Hardware-specific knobs for the manipulator backend. DOF / EEF
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
                positions=np.zeros(self.driver.get_num_eef_dofs(), dtype=np.float64),
            )
        if velocities is None:
            velocities = EEFVelocities(
                header=header,
                velocities=np.zeros(self.driver.get_num_eef_dofs(), dtype=np.float64),
            )
        return EEFState(
            header=header,
            eef_positions=positions,
            eef_velocities=velocities,
        )
