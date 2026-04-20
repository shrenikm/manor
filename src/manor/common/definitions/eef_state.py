"""
End-effector state (generalized positions + velocities) at a single time instant.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.lcmtypes.lcmt_eef_state import lcmt_eef_state
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


@attr.frozen
class EEFState(DefinitionBase):
    """
    EEF generalized positions and velocities at the same time instant.
    """

    header: TimestampHeader
    eef_positions: EEFPositions
    eef_velocities: EEFVelocities

    LCM_CLASS: ClassVar[type] = lcmt_eef_state
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> Any:
        return load_versioned_schema("eef_state.capnp").VersionedEEFState

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        self.eef_positions._to_capnp_current(builder.init("eefPositions"))
        self.eef_velocities._to_capnp_current(builder.init("eefVelocities"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            eef_positions=EEFPositions._from_capnp_v1(reader.eefPositions),
            eef_velocities=EEFVelocities._from_capnp_v1(reader.eefVelocities),
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
