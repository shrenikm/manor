"""
End-effector state (generalized positions + velocities) at a single time instant.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.lcmtypes.lcmt_eef_state import lcmt_eef_state
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    EEF_POSITIONS = "eefPositions"
    EEF_VELOCITIES = "eefVelocities"
    HEADER = "header"


@attr.frozen
class EEFState(DefinitionBase):
    """
    EEF generalized positions and velocities at the same time instant.
    """

    header: TimestampHeader
    eef_positions: EEFPositions
    eef_velocities: EEFVelocities

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("eef_state.capnp").VersionedEEFState

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_eef_state

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        self.eef_positions.to_versioned_capnp(builder.init(_CapnpField.EEF_POSITIONS))
        self.eef_velocities.to_versioned_capnp(builder.init(_CapnpField.EEF_VELOCITIES))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            eef_positions=EEFPositions.from_versioned_capnp(reader.eefPositions),
            eef_velocities=EEFVelocities.from_versioned_capnp(reader.eefVelocities),
        )

    @override
    def to_lcm_message(self) -> lcmt_eef_state:
        msg = lcmt_eef_state()
        msg.header = self.header.to_lcm_message()
        msg.eef_positions = self.eef_positions.to_lcm_message()
        msg.eef_velocities = self.eef_velocities.to_lcm_message()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            eef_positions=EEFPositions.from_lcm_message(msg.eef_positions),
            eef_velocities=EEFVelocities.from_lcm_message(msg.eef_velocities),
        )

    @classmethod
    @override
    def construct_default(cls, num_eef_dofs: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            eef_positions=EEFPositions.construct_default(num_eef_dofs=num_eef_dofs),
            eef_velocities=EEFVelocities.construct_default(num_eef_dofs=num_eef_dofs),
        )
