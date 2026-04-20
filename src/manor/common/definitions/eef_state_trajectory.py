"""
Time-indexed trajectory of end-effector state.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_eef_state_trajectory import (
    lcmt_eef_state_trajectory,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    CapnpStructSchema,
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    EEF_POSITIONS_ARRAY = "eefPositionsArray"
    EEF_VELOCITIES_ARRAY = "eefVelocitiesArray"
    HEADER = "header"
    TIMES = "times"


@attr.frozen
class EEFStateTrajectory(DefinitionBase):
    """
    A trajectory of EEF state. eef_positions_array and eef_velocities_array have
    shape (num_steps, num_coords); times has shape (num_steps,).
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    eef_positions_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    eef_velocities_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("eef_state_trajectory.capnp").VersionedEEFStateTrajectory

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_eef_state_trajectory

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.times, builder.init(_CapnpField.TIMES))
        ndarray_to_float64_array(self.eef_positions_array, builder.init(_CapnpField.EEF_POSITIONS_ARRAY))
        ndarray_to_float64_array(self.eef_velocities_array, builder.init(_CapnpField.EEF_VELOCITIES_ARRAY))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            times=float64_array_to_ndarray(reader.times),
            eef_positions_array=float64_array_to_ndarray(reader.eefPositionsArray),
            eef_velocities_array=float64_array_to_ndarray(reader.eefVelocitiesArray),
        )

    @override
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
    @override
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
