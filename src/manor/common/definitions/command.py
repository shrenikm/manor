"""
Robot command.

The controller (Kyber) emits a Command to the hardware interface (Talos).
Every Command carries a JointCommand (the joint-level setpoint Talos
actuates) and may optionally carry an EECommand (a gripper-side
setpoint). The joint side is required because the arm always needs a
target on every tick; the gripper side is optional because gripper
control is not always needed and many policies leave it untouched.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.lcmtypes.lcmt_command import lcmt_command
from manor.common.definitions.lcmtypes.lcmt_ee_command import lcmt_ee_command
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    HEADER = "header"
    JOINT_COMMAND = "jointCommand"
    EE_COMMAND = "eeCommand"


@attr.frozen
class Command(DefinitionBase):
    """
    A hardware-facing command. Carries a required JointCommand for the
    arm and an optional EECommand for the gripper.
    """

    header: TimestampHeader
    joint_command: JointCommand
    ee_command: EECommand | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("command.capnp").VersionedCommand

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_command

    @override
    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        self.joint_command.to_versioned_capnp(builder.init(_CapnpField.JOINT_COMMAND))
        if self.ee_command is None:
            builder.eeCommand.none = None
        else:
            self.ee_command.to_versioned_capnp(builder.eeCommand.init("some"))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        ee_command = (
            EECommand.from_versioned_capnp(reader.eeCommand.some) if reader.eeCommand.which() == "some" else None
        )
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            joint_command=JointCommand.from_versioned_capnp(reader.jointCommand),
            ee_command=ee_command,
        )

    @override
    def to_lcm_message(self) -> lcmt_command:
        msg = lcmt_command()
        msg.header = self.header.to_lcm_message()
        msg.joint_command = self.joint_command.to_lcm_message()
        msg.has_ee_command = self.ee_command is not None
        msg.ee_command = self.ee_command.to_lcm_message() if self.ee_command is not None else lcmt_ee_command()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            joint_command=JointCommand.from_lcm_message(msg.joint_command),
            ee_command=EECommand.from_lcm_message(msg.ee_command) if msg.has_ee_command else None,
        )

    @classmethod
    @override
    def construct_default(cls, num_joints: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            joint_command=JointCommand.construct_default(num_joints=num_joints),
        )
