"""
Time-indexed trajectory of end-effector generalized velocities.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np

from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions._capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.eef_velocities_trajectory_t import eef_velocities_trajectory_t

_CAPNP = load_versioned_schema("eef_velocities_trajectory")


@attr.frozen
class EEFVelocitiesTrajectory(DefinitionBase):
    """
    A trajectory of EEF generalized velocities.
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    eef_velocities_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedEefVelocitiesTrajectory
    LCM_CLASS: ClassVar[type] = eef_velocities_trajectory_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.times, builder.init("times"))
        ndarray_to_float64_array(self.eef_velocities_array, builder.init("eefVelocitiesArray"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            times=float64_array_to_ndarray(reader.times),
            eef_velocities_array=float64_array_to_ndarray(reader.eefVelocitiesArray),
        )

    def to_lcm_message(self) -> eef_velocities_trajectory_t:
        msg = eef_velocities_trajectory_t()
        msg.header = self.header.to_lcm_message()
        arr = np.ascontiguousarray(self.eef_velocities_array, dtype=np.float64)
        msg.num_steps = int(arr.shape[0])
        msg.num_velocities = int(arr.shape[1]) if arr.ndim > 1 else 0
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.eef_velocities_array = arr.tolist()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            eef_velocities_array=np.array(msg.eef_velocities_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_velocities
            ),
        )
