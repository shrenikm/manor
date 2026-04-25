"""
End-effector Cartesian pose (translation + quaternion orientation).
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.custom_types import NpVector3f64, NpVector4f64
from manor.common.definitions.lcmtypes.lcmt_eef_pose import lcmt_eef_pose
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import (
    CapnpStructSchema,
    float64_array_to_ndarray,
    load_versioned_schema,
    ndarray_to_float64_array,
)
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    HEADER = "header"
    ORIENTATION = "orientation"
    TRANSLATION = "translation"


@attr.frozen
class EEFPose(DefinitionBase):
    """
    Pose of the EEF control point in world (or base) frame.

    translation is (x, y, z) in meters.
    orientation is a unit quaternion (w, x, y, z).
    """

    header: TimestampHeader
    translation: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    orientation: NpVector4f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("eef_pose.capnp").VersionedEEFPose

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_eef_pose

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.translation, builder.init(_CapnpField.TRANSLATION))
        ndarray_to_float64_array(self.orientation, builder.init(_CapnpField.ORIENTATION))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            translation=float64_array_to_ndarray(reader.translation),
            orientation=float64_array_to_ndarray(reader.orientation),
        )

    @override
    def to_lcm_message(self) -> lcmt_eef_pose:
        msg = lcmt_eef_pose()
        msg.header = self.header.to_lcm_message()
        msg.translation = self.translation.astype(np.float64).tolist()
        msg.orientation = self.orientation.astype(np.float64).tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            translation=np.array(msg.translation, dtype=np.float64),
            orientation=np.array(msg.orientation, dtype=np.float64),
        )

    @classmethod
    @override
    def construct_default(cls) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            translation=np.zeros(3, dtype=np.float64),
            orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        )
