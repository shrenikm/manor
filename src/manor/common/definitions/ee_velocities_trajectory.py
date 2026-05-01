"""
Time-indexed trajectory of end-effector generalized velocities.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.attrs_utils import is_2d_array, is_non_decreasing_1d
from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_ee_velocities_trajectory import (
    lcmt_ee_velocities_trajectory,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    CapnpStructSchema,
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.exceptions import InvalidDefinitionError


class _CapnpField(StrEnum):
    EE_VELOCITIES_ARRAY = "eeVelocitiesArray"
    HEADER = "header"
    TIMES = "times"


@attr.frozen
class EEVelocitiesTrajectory(DefinitionBase):
    """
    A trajectory of EE generalized velocities.
    """

    header: TimestampHeader
    times: TimesVector = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_non_decreasing_1d(),
    )
    ee_velocities_array: NpMatrixNMf64 = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_2d_array(),
    )

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    def __attrs_post_init__(self) -> None:
        if self.times.shape[0] == 0:
            raise InvalidDefinitionError(
                "EEVelocitiesTrajectory must contain at least one step (use None for an absent trajectory)"
            )
        if self.times.shape[0] != self.ee_velocities_array.shape[0]:
            raise InvalidDefinitionError(
                f"EEVelocitiesTrajectory.times length ({self.times.shape[0]}) "
                f"must match ee_velocities_array rows ({self.ee_velocities_array.shape[0]})"
            )

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("ee_velocities_trajectory.capnp").VersionedEEVelocitiesTrajectory

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_ee_velocities_trajectory

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.times, builder.init(_CapnpField.TIMES))
        ndarray_to_float64_array(self.ee_velocities_array, builder.init(_CapnpField.EE_VELOCITIES_ARRAY))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            times=float64_array_to_ndarray(reader.times),
            ee_velocities_array=float64_array_to_ndarray(reader.eeVelocitiesArray),
        )

    @override
    def to_lcm_message(self) -> lcmt_ee_velocities_trajectory:
        msg = lcmt_ee_velocities_trajectory()
        msg.header = self.header.to_lcm_message()
        arr = np.ascontiguousarray(self.ee_velocities_array, dtype=np.float64)
        msg.num_steps = int(arr.shape[0])
        msg.num_velocities = int(arr.shape[1]) if arr.ndim > 1 else 0
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.ee_velocities_array = arr.tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            ee_velocities_array=np.array(msg.ee_velocities_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_velocities
            ),
        )

    @classmethod
    @override
    def construct_default(cls, num_steps: int = 1, num_ee_dofs: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            times=np.zeros(num_steps, dtype=np.float64),
            ee_velocities_array=np.zeros((num_steps, num_ee_dofs), dtype=np.float64),
        )
