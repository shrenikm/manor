"""
Time-indexed trajectory of end-effector generalized positions.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_eef_positions_trajectory import (
    lcmt_eef_positions_trajectory,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import DefinitionBase


@attr.frozen
class EEFPositionsTrajectory(DefinitionBase):
    """
    A trajectory of EEF generalized positions.
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    eef_positions_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    LCM_CLASS: ClassVar[type] = lcmt_eef_positions_trajectory
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> Any:
        return load_versioned_schema(
            "eef_positions_trajectory.capnp"
        ).VersionedEEFPositionsTrajectory

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.times, builder.init("times"))
        ndarray_to_float64_array(self.eef_positions_array, builder.init("eefPositionsArray"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            times=float64_array_to_ndarray(reader.times),
            eef_positions_array=float64_array_to_ndarray(reader.eefPositionsArray),
        )

    @override
    def to_lcm_message(self) -> lcmt_eef_positions_trajectory:
        msg = lcmt_eef_positions_trajectory()
        msg.header = self.header.to_lcm_message()
        arr = np.ascontiguousarray(self.eef_positions_array, dtype=np.float64)
        msg.num_steps = int(arr.shape[0])
        msg.num_positions = int(arr.shape[1]) if arr.ndim > 1 else 0
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.eef_positions_array = arr.tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            eef_positions_array=np.array(msg.eef_positions_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_positions
            ),
        )
