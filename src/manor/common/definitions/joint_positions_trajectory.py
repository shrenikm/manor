"""
Time-indexed trajectory of joint positions.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.attrs_utils import is_2d_array, is_non_decreasing_1d
from manor.common.custom_types import NpMatrixNMf64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_joint_positions_trajectory import (
    lcmt_joint_positions_trajectory,
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
    HEADER = "header"
    JOINT_POSITIONS_ARRAY = "jointPositionsArray"
    TIMES = "times"


@attr.frozen
class JointPositionsTrajectory(DefinitionBase):
    """
    A trajectory of joint positions.

    joint_positions_array has shape (num_steps, num_joints) and times has shape
    (num_steps,), one-to-one with the rows of joint_positions_array.
    """

    header: TimestampHeader
    times: TimesVector = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_non_decreasing_1d(),
    )
    joint_positions_array: NpMatrixNMf64 = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_2d_array(),
    )

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    def __attrs_post_init__(self) -> None:
        if self.times.shape[0] == 0:
            raise InvalidDefinitionError(
                "JointPositionsTrajectory must contain at least one step (use None for an absent trajectory)"
            )
        if self.times.shape[0] != self.joint_positions_array.shape[0]:
            raise InvalidDefinitionError(
                f"JointPositionsTrajectory.times length ({self.times.shape[0]}) "
                f"must match joint_positions_array rows ({self.joint_positions_array.shape[0]})"
            )

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("joint_positions_trajectory.capnp").VersionedJointPositionsTrajectory

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_joint_positions_trajectory

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.times, builder.init(_CapnpField.TIMES))
        ndarray_to_float64_array(self.joint_positions_array, builder.init(_CapnpField.JOINT_POSITIONS_ARRAY))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            times=float64_array_to_ndarray(reader.times),
            joint_positions_array=float64_array_to_ndarray(reader.jointPositionsArray),
        )

    @override
    def to_lcm_message(self) -> lcmt_joint_positions_trajectory:
        msg = lcmt_joint_positions_trajectory()
        msg.header = self.header.to_lcm_message()
        arr = np.ascontiguousarray(self.joint_positions_array, dtype=np.float64)
        msg.num_steps = int(arr.shape[0])
        msg.num_joints = int(arr.shape[1]) if arr.ndim > 1 else 0
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.joint_positions_array = arr.tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            joint_positions_array=np.array(msg.joint_positions_array, dtype=np.float64).reshape(
                msg.num_steps, msg.num_joints
            ),
        )

    @classmethod
    @override
    def construct_default(cls, num_steps: int = 1, num_joints: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            times=np.zeros(num_steps, dtype=np.float64),
            joint_positions_array=np.zeros((num_steps, num_joints), dtype=np.float64),
        )
