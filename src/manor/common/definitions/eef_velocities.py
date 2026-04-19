"""
End-effector generalized velocities at a single time instant.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np

from manor.common.custom_types import NpVectorNf64
from manor.common.definitions._capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.eef_velocities_t import eef_velocities_t

_CAPNP = load_versioned_schema("eef_velocities")


@attr.frozen
class EEFVelocities(DefinitionBase):
    """
    EEF generalized velocities.
    """

    header: TimestampHeader
    velocities: NpVectorNf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedEefVelocities
    LCM_CLASS: ClassVar[type] = eef_velocities_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.velocities, builder.init("velocities"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            velocities=float64_array_to_ndarray(reader.velocities),
        )

    def to_lcm_message(self) -> eef_velocities_t:
        msg = eef_velocities_t()
        msg.header = self.header.to_lcm_message()
        msg.num_velocities = int(self.velocities.size)
        msg.velocities = self.velocities.astype(np.float64).tolist()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            velocities=np.array(msg.velocities, dtype=np.float64),
        )
