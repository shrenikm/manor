"""
Time-indexed trajectory of joint velocities.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
import numpy as np

from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_joint_velocities_trajectory import (
    lcmt_joint_velocities_trajectory,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import IDefinition

_CAPNP = load_versioned_schema("joint_velocities_trajectory")


@attr.frozen
class JointVelocitiesTrajectory(IDefinition):
    """
    A trajectory of joint velocities.

    joint_velocities_array has shape (num_steps, num_joints) and times has shape
    (num_steps,).
    """

    header: TimestampHeader
    times: TimesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    joint_velocities_array: NpMatrixNMf64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedJointVelocitiesTrajectory
    LCM_CLASS: ClassVar[type] = lcmt_joint_velocities_trajectory
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        ndarray_to_float64_array(self.times, builder.init("times"))
        ndarray_to_float64_array(
            self.joint_velocities_array, builder.init("jointVelocitiesArray")
        )

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            times=float64_array_to_ndarray(reader.times),
            joint_velocities_array=float64_array_to_ndarray(reader.jointVelocitiesArray),
        )

    def to_lcm_message(self) -> lcmt_joint_velocities_trajectory:
        msg = lcmt_joint_velocities_trajectory()
        msg.header = self.header.to_lcm_message()
        arr = np.ascontiguousarray(self.joint_velocities_array, dtype=np.float64)
        msg.num_steps = int(arr.shape[0])
        msg.num_joints = int(arr.shape[1]) if arr.ndim > 1 else 0
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.joint_velocities_array = arr.tolist()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            joint_velocities_array=np.array(msg.joint_velocities_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_joints
            ),
        )
