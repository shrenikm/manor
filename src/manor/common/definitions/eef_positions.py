"""
End-effector generalized positions at a single time instant.

EEF positions are generalized coordinates: for a parallel gripper that's the
finger opening; for a dexterous hand they are finger-joint positions, etc.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np

from manor.common.custom_types import NpVectorNf64
from manor.common.definitions.lcmtypes.lcmt_eef_positions import lcmt_eef_positions
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import IDefinition

_CAPNP = load_versioned_schema("eef_positions")


@attr.frozen
class EEFPositions(IDefinition):
    """
    EEF generalized positions.
    """

    header: TimestampHeader
    positions: NpVectorNf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedEefPositions
    LCM_CLASS: ClassVar[type] = lcmt_eef_positions
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

    def to_lcm_message(self) -> lcmt_eef_positions:
        msg = lcmt_eef_positions()
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
