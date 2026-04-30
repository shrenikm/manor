"""
End-effector state (generalized positions + velocities) at a single time instant.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.lcmtypes.lcmt_ee_state import lcmt_ee_state
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    EE_POSITIONS = "eePositions"
    EE_VELOCITIES = "eeVelocities"
    HEADER = "header"


@attr.frozen
class EEState(DefinitionBase):
    """
    EE generalized positions and velocities at the same time instant.
    """

    header: TimestampHeader
    ee_positions: EEPositions
    ee_velocities: EEVelocities

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("ee_state.capnp").VersionedEEState

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_ee_state

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        self.ee_positions.to_versioned_capnp(builder.init(_CapnpField.EE_POSITIONS))
        self.ee_velocities.to_versioned_capnp(builder.init(_CapnpField.EE_VELOCITIES))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            ee_positions=EEPositions.from_versioned_capnp(reader.eePositions),
            ee_velocities=EEVelocities.from_versioned_capnp(reader.eeVelocities),
        )

    @override
    def to_lcm_message(self) -> lcmt_ee_state:
        msg = lcmt_ee_state()
        msg.header = self.header.to_lcm_message()
        msg.ee_positions = self.ee_positions.to_lcm_message()
        msg.ee_velocities = self.ee_velocities.to_lcm_message()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            ee_positions=EEPositions.from_lcm_message(msg.ee_positions),
            ee_velocities=EEVelocities.from_lcm_message(msg.ee_velocities),
        )

    @classmethod
    @override
    def construct_default(cls, num_ee_dofs: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            ee_positions=EEPositions.construct_default(num_ee_dofs=num_ee_dofs),
            ee_velocities=EEVelocities.construct_default(num_ee_dofs=num_ee_dofs),
        )
