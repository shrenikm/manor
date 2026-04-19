"""
Time-indexed trajectory of end-effector state.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np

from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_eef_state_trajectory import (
    lcmt_eef_state_trajectory,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import IDefinition

_CAPNP = load_versioned_schema("eef_state_trajectory")


@attr.frozen
class EEFStateTrajectory(IDefinition):
    """
    A trajectory of EEF state. eef_positions_array and eef_velocities_array have
    shape (num_steps, num_coords); times has shape (num_steps,).
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    eef_positions_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    eef_velocities_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedEefStateTrajectory
    LCM_CLASS: ClassVar[type] = lcmt_eef_state_trajectory
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.times, builder.init("times"))
        ndarray_to_float64_array(self.eef_positions_array, builder.init("eefPositionsArray"))
        ndarray_to_float64_array(self.eef_velocities_array, builder.init("eefVelocitiesArray"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            times=float64_array_to_ndarray(reader.times),
            eef_positions_array=float64_array_to_ndarray(reader.eefPositionsArray),
            eef_velocities_array=float64_array_to_ndarray(reader.eefVelocitiesArray),
        )

    def to_lcm_message(self) -> lcmt_eef_state_trajectory:
        msg = lcmt_eef_state_trajectory()
        msg.header = self.header.to_lcm_message()
        pos = np.ascontiguousarray(self.eef_positions_array, dtype=np.float64)
        vel = np.ascontiguousarray(self.eef_velocities_array, dtype=np.float64)
        msg.num_steps = int(pos.shape[0])
        msg.num_coords = int(pos.shape[1]) if pos.ndim > 1 else 0
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.eef_positions_array = pos.tolist()
        msg.eef_velocities_array = vel.tolist()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            eef_positions_array=np.array(msg.eef_positions_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_coords
            ),
            eef_velocities_array=np.array(msg.eef_velocities_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_coords
            ),
        )
