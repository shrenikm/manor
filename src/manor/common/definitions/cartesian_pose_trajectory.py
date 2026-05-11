"""
Time-indexed trajectory of end-effector pose.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.attrs_utils import is_2d_array, is_non_decreasing_1d
from manor.common.custom_types import NpMatrixN3f64, NpMatrixN4f64, TimesVector
from manor.common.definitions.lcmtypes.lcmt_cartesian_pose_trajectory import (
    lcmt_cartesian_pose_trajectory,
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
    ORIENTATIONS_ARRAY = "orientationsArray"
    TIMES = "times"
    TRANSLATIONS_ARRAY = "translationsArray"


@attr.frozen
class CartesianPoseTrajectory(DefinitionBase):
    """
    A trajectory of Cartesian poses.

    translations_array has shape (num_steps, 3); orientations_array has shape
    (num_steps, 4). times has shape (num_steps,).
    """

    header: TimestampHeader
    times: TimesVector = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_non_decreasing_1d(),
    )
    translations_array: NpMatrixN3f64 = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_2d_array(expected_cols=3),
    )
    orientations_array: NpMatrixN4f64 = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_2d_array(expected_cols=4),
    )

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    def __attrs_post_init__(self) -> None:
        n = self.times.shape[0]
        if n == 0:
            raise InvalidDefinitionError(
                "CartesianPoseTrajectory must contain at least one step (use None for an absent trajectory)"
            )
        if self.translations_array.shape[0] != n:
            raise InvalidDefinitionError(
                f"CartesianPoseTrajectory.times length ({n}) "
                f"must match translations_array rows ({self.translations_array.shape[0]})"
            )
        if self.orientations_array.shape[0] != n:
            raise InvalidDefinitionError(
                f"CartesianPoseTrajectory.times length ({n}) "
                f"must match orientations_array rows ({self.orientations_array.shape[0]})"
            )

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("cartesian_pose_trajectory.capnp").VersionedCartesianPoseTrajectory

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_cartesian_pose_trajectory

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.times, builder.init(_CapnpField.TIMES))
        ndarray_to_float64_array(self.translations_array, builder.init(_CapnpField.TRANSLATIONS_ARRAY))
        ndarray_to_float64_array(self.orientations_array, builder.init(_CapnpField.ORIENTATIONS_ARRAY))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            times=float64_array_to_ndarray(reader.times),
            translations_array=float64_array_to_ndarray(reader.translationsArray),
            orientations_array=float64_array_to_ndarray(reader.orientationsArray),
        )

    @override
    def to_lcm_message(self) -> lcmt_cartesian_pose_trajectory:
        msg = lcmt_cartesian_pose_trajectory()
        msg.header = self.header.to_lcm_message()
        tr = np.ascontiguousarray(self.translations_array, dtype=np.float64)
        ori = np.ascontiguousarray(self.orientations_array, dtype=np.float64)
        msg.num_steps = int(tr.shape[0])
        msg.times = np.ascontiguousarray(self.times, dtype=np.float64).tolist()
        msg.translations_array = tr.tolist()
        msg.orientations_array = ori.tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            times=np.array(msg.times, dtype=np.float64),
            translations_array=np.array(msg.translations_array, dtype=np.float64).reshape(msg.num_steps, 3),
            orientations_array=np.array(msg.orientations_array, dtype=np.float64).reshape(msg.num_steps, 4),
        )

    @classmethod
    @override
    def construct_default(cls, num_steps: int = 1) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            times=np.zeros(num_steps, dtype=np.float64),
            translations_array=np.zeros((num_steps, 3), dtype=np.float64),
            orientations_array=np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64), (num_steps, 1)),
        )
