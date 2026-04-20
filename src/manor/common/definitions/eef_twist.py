"""
End-effector spatial twist (linear and angular velocity).
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.custom_types import NpVector3f64
from manor.common.definitions.lcmtypes.lcmt_eef_twist import lcmt_eef_twist
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import DefinitionBase


@attr.frozen
class EEFTwist(DefinitionBase):
    """
    EEF spatial twist.

    linear is (vx, vy, vz) m/s. angular is (wx, wy, wz) rad/s.
    """

    header: TimestampHeader
    linear: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    angular: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> Any:
        return load_versioned_schema("eef_twist.capnp").VersionedEEFTwist

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_eef_twist

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.linear, builder.init("linear"))
        ndarray_to_float64_array(self.angular, builder.init("angular"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            linear=float64_array_to_ndarray(reader.linear),
            angular=float64_array_to_ndarray(reader.angular),
        )

    @override
    def to_lcm_message(self) -> lcmt_eef_twist:
        msg = lcmt_eef_twist()
        msg.header = self.header.to_lcm_message()
        msg.linear = self.linear.astype(np.float64).tolist()
        msg.angular = self.angular.astype(np.float64).tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            linear=np.array(msg.linear, dtype=np.float64),
            angular=np.array(msg.angular, dtype=np.float64),
        )
