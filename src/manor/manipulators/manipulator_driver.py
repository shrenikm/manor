"""
Abstract interface for a manipulator hardware driver.

A driver wraps the vendor SDK / network protocol for a real
manipulator. Talos consumes this interface to read state from and write
commands to the robot, while remaining agnostic to which manipulator
(and therefore which SDK) is on the other end.

Read methods always return a freshly stamped definition. Write methods
dispatch the latest command to the underlying SDK. The end-effector
read/write methods may return / accept ``None`` for manipulators whose
EE has no continuous-controllable DOFs (e.g. a vacuum gripper).
"""

from __future__ import annotations

import abc

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities


class IManipulatorDriver(abc.ABC):
    """
    Hardware-driver interface for a single manipulator.
    """

    @abc.abstractmethod
    def get_num_dof(self) -> int:
        """
        Arm degrees of freedom (excluding the EE). Mirrors the same
        accessor on ``IManipulatorModel`` so the driver can be queried
        for sizing without having to thread the model alongside it.
        """
        ...

    @abc.abstractmethod
    def get_num_ee_dofs(self) -> int:
        """
        EE generalized-DOF count (size of EEPositions / EEVelocities
        vectors emitted by this driver). Same semantics as on
        ``IManipulatorModel.get_num_ee_dofs``.
        """
        ...

    @abc.abstractmethod
    def prime(self) -> None:
        """
        Prepare the arm for use after boot-up or reset. Connects to the
        SDK, clears errors, enables motion, and brings the manipulator
        into a state ready to accept commands.
        """
        ...

    @abc.abstractmethod
    def unprime(self) -> None:
        """
        Tear down before shutdown or reset. Stops motion, releases
        control, and disconnects the SDK so a subsequent ``prime`` can
        start cleanly.
        """
        ...

    @abc.abstractmethod
    def read_joint_positions(self) -> JointPositions: ...

    @abc.abstractmethod
    def read_ee_positions(self) -> EEPositions | None: ...

    @abc.abstractmethod
    def read_joint_velocities(self) -> JointVelocities: ...

    @abc.abstractmethod
    def read_ee_velocities(self) -> EEVelocities | None: ...

    @abc.abstractmethod
    def write_joint_positions(self, joint_positions: JointPositions) -> None: ...

    @abc.abstractmethod
    def write_ee_positions(self, ee_positions: EEPositions) -> None: ...

    @abc.abstractmethod
    def write_joint_velocities(self, joint_velocities: JointVelocities) -> None: ...

    @abc.abstractmethod
    def write_ee_velocities(self, ee_velocities: EEVelocities) -> None: ...
