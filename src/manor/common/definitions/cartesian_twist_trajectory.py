"""
Time-indexed trajectory of end-effector spatial twist.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.custom_types import NpMatrixN3f64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_cartesian_twist_trajectory import (
    lcmt_cartesian_twist_trajectory,
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
    ANGULAR_ARRAY = "angularArray"
    HEADER = "header"
    LINEAR_ARRAY = "linearArray"
    TIMES = "times"


@attr.frozen
class CartesianTwistTrajectory(DefinitionBase):
    """
    A trajectory of End-effector spatial twists.

    linear_array has shape (num_steps, 3); angular_array has shape (num_steps, 3).
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    linear_array: NpMatrixN3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    angular_array: NpMatrixN3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("cartesian_twist_trajectory.capnp").VersionedCartesianTwistTrajectory

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_cartesian_twist_trajectory

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.times, builder.init(_CapnpField.TIMES))
        ndarray_to_float64_array(self.linear_array, builder.init(_CapnpField.LINEAR_ARRAY))
        ndarray_to_float64_array(self.angular_array, builder.init(_CapnpField.ANGULAR_ARRAY))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            times=float64_array_to_ndarray(reader.times),
            linear_array=float64_array_to_ndarray(reader.linearArray),
            angular_array=float64_array_to_ndarray(reader.angularArray),
        )

    @override
    def to_lcm_message(self) -> lcmt_cartesian_twist_trajectory:
        msg = lcmt_cartesian_twist_trajectory()
        msg.header = self.header.to_lcm_message()
        lin = np.ascontiguousarray(self.linear_array, dtype=np.float64)
        ang = np.ascontiguousarray(self.angular_array, dtype=np.float64)
        msg.num_steps = int(lin.shape[0])
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.linear_array = lin.tolist()
        msg.angular_array = ang.tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            linear_array=np.array(msg.linear_array, dtype=np.float64).reshape(msg.num_steps, 3),
            angular_array=np.array(msg.angular_array, dtype=np.float64).reshape(msg.num_steps, 3),
        )

    @classmethod
    @override
    def construct_default(cls, num_steps: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            times=np.zeros(num_steps, dtype=np.float64),
            linear_array=np.zeros((num_steps, 3), dtype=np.float64),
            angular_array=np.zeros((num_steps, 3), dtype=np.float64),
        )
