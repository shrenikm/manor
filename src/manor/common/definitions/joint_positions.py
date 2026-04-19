"""
Joint positions at a single time instant.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np
from manor_lcm.joint_positions_t import joint_positions_t

from manor.common.custom_types import JointPositionsVector
from manor.common.definitions._capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.timestamp_header import TimestampHeader

_CAPNP = load_versioned_schema("joint_positions")


@attr.frozen
class JointPositions(DefinitionBase):
    """
    Per-joint generalized positions.
    """

    header: TimestampHeader
    positions: JointPositionsVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedJointPositions
    LCM_CLASS: ClassVar[type] = joint_positions_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.positions, builder.init("positions"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            positions=float64_array_to_ndarray(reader.positions),
        )

    def to_lcm_message(self) -> joint_positions_t:
        msg = joint_positions_t()
        msg.header = self.header.to_lcm_message()
        msg.num_positions = int(self.positions.size)
        msg.positions = self.positions.astype(np.float64).tolist()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            positions=np.array(msg.positions, dtype=np.float64),
        )
