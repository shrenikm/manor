"""
Joint state (positions + velocities) at a single time instant.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.lcmtypes.lcmt_joint_state import lcmt_joint_state
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import load_versioned_schema
from manor.common.definitions.utils.interfaces import IDefinition

_CAPNP = load_versioned_schema("joint_state")


@attr.frozen
class JointState(IDefinition):
    """
    Joint positions and velocities at the same time instant.
    """

    header: TimestampHeader
    joint_positions: JointPositions
    joint_velocities: JointVelocities

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedJointState
    LCM_CLASS: ClassVar[type] = lcmt_joint_state
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        self.joint_positions._to_capnp_current(builder.init("jointPositions"))
        self.joint_velocities._to_capnp_current(builder.init("jointVelocities"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            joint_positions=JointPositions._from_capnp_v1(reader.jointPositions),
            joint_velocities=JointVelocities._from_capnp_v1(reader.jointVelocities),
        )

    def to_lcm_message(self) -> lcmt_joint_state:
        msg = lcmt_joint_state()
        msg.header = self.header.to_lcm_message()
        msg.joint_positions = self.joint_positions.to_lcm_message()
        msg.joint_velocities = self.joint_velocities.to_lcm_message()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            joint_positions=JointPositions.from_lcm_message(msg.joint_positions),
            joint_velocities=JointVelocities.from_lcm_message(msg.joint_velocities),
        )
