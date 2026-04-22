"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK (xarm-python-sdk for the Lite6). For now the
SDK calls are mocked: the backend stores the latest command and returns
zero-valued state with a fresh system-time header.
"""

from __future__ import annotations

import attr

from manor.common.aegis.defaults import default_eef_state, default_joint_state, system_time_header
from manor.common.definitions.command import Command
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.joint_state import JointState


@attr.frozen
class HardwareManipulatorBackendConfig:
    """
    Connection info for the real manipulator.
    """

    ip_address: str = ""
    num_joints: int = 6
    num_eef_dofs: int = 1


@attr.define
class HardwareManipulatorBackend:
    """
    ManipulatorBackend that talks to a real robot. SDK calls are stubbed.
    """

    config: HardwareManipulatorBackendConfig = attr.field(factory=HardwareManipulatorBackendConfig)
    _sdk: object | None = attr.field(default=None, init=False)
    _latest_command: Command | None = attr.field(default=None, init=False)

    def start(self) -> None:
        # TODO: connect to the robot, enable motion, clear errors, go to a home pose.
        self._sdk = object()

    def stop(self) -> None:
        # TODO: disable motion, release control, disconnect.
        self._sdk = None

    def send_command(self, command: Command) -> None:
        # TODO: dispatch on command variant and call the appropriate SDK method.
        self._latest_command = command

    def read_joint_state(self) -> JointState:
        # TODO: query the SDK for joint angles + velocities.
        return attr.evolve(default_joint_state(self.config.num_joints), header=system_time_header())

    def read_eef_state(self) -> EEFState:
        # TODO: query the SDK for gripper state.
        return attr.evolve(default_eef_state(self.config.num_eef_dofs), header=system_time_header())
