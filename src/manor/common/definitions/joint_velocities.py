"""
Joint velocities at a single time instant.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.attrs_utils import is_1d_array
from manor.common.custom_types import JointVelocitiesVector
from manor.common.definitions.lcmtypes.lcmt_joint_velocities import lcmt_joint_velocities
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
    VELOCITIES = "velocities"


@attr.frozen
class JointVelocities(DefinitionBase):
    """
    Per-joint generalized velocities.
    """

    header: TimestampHeader
    velocities: JointVelocitiesVector = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_1d_array(),
    )

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("joint_velocities.capnp").VersionedJointVelocities

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_joint_velocities

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.velocities, builder.init(_CapnpField.VELOCITIES))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            velocities=float64_array_to_ndarray(reader.velocities),
        )

    @override
    def to_lcm_message(self) -> lcmt_joint_velocities:
        msg = lcmt_joint_velocities()
        msg.header = self.header.to_lcm_message()
        msg.num_velocities = int(self.velocities.size)
        msg.velocities = self.velocities.astype(np.float64).tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            velocities=np.array(msg.velocities, dtype=np.float64),
        )

    @classmethod
    @override
    def construct_default(cls, num_joints: int = 0) -> Self:
        return cls(header=TimestampHeader.construct_default(), velocities=np.zeros(num_joints, dtype=np.float64))
