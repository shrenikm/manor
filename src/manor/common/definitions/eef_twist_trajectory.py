"""
Time-indexed trajectory of end-effector spatial twist.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np

from manor.common.custom_types import NpMatrixN3f64, TimesVector
from manor.common.definitions._capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.eef_twist_trajectory_t import eef_twist_trajectory_t

_CAPNP = load_versioned_schema("eef_twist_trajectory")


@attr.frozen
class EEFTwistTrajectory(DefinitionBase):
    """
    A trajectory of EEF spatial twists.

    linear_array has shape (num_steps, 3); angular_array has shape (num_steps, 3).
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    linear_array: NpMatrixN3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    angular_array: NpMatrixN3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedEefTwistTrajectory
    LCM_CLASS: ClassVar[type] = eef_twist_trajectory_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.times, builder.init("times"))
        ndarray_to_float64_array(self.linear_array, builder.init("linearArray"))
        ndarray_to_float64_array(self.angular_array, builder.init("angularArray"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            times=float64_array_to_ndarray(reader.times),
            linear_array=float64_array_to_ndarray(reader.linearArray),
            angular_array=float64_array_to_ndarray(reader.angularArray),
        )

    def to_lcm_message(self) -> eef_twist_trajectory_t:
        msg = eef_twist_trajectory_t()
        msg.header = self.header.to_lcm_message()
        lin = np.ascontiguousarray(self.linear_array, dtype=np.float64)
        ang = np.ascontiguousarray(self.angular_array, dtype=np.float64)
        msg.num_steps = int(lin.shape[0])
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.linear_array = lin.tolist()
        msg.angular_array = ang.tolist()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            linear_array=np.array(msg.linear_array, dtype=np.float64).reshape(msg.num_steps, 3),
            angular_array=np.array(msg.angular_array, dtype=np.float64).reshape(msg.num_steps, 3),
        )
