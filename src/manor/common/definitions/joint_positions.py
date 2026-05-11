"""
Joint positions at a single time instant.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr
import numpy as np

from manor.common.attrs_utils import is_1d_array
from manor.common.custom_types import JointPositionsVector
from manor.common.definitions.lcmtypes.lcmt_joint_positions import lcmt_joint_positions
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
    POSITIONS = "positions"


@attr.frozen
class JointPositions(DefinitionBase):
    """
    Per-joint generalized positions.
    """

    header: TimestampHeader
    positions: JointPositionsVector = attr.field(
        eq=attr.cmp_using(eq=np.array_equal),
        validator=is_1d_array(),
    )

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("joint_positions.capnp").VersionedJointPositions

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_joint_positions

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        ndarray_to_float64_array(self.positions, builder.init(_CapnpField.POSITIONS))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            positions=float64_array_to_ndarray(reader.positions),
        )

    @override
    def to_lcm_message(self) -> lcmt_joint_positions:
        msg = lcmt_joint_positions()
        msg.header = self.header.to_lcm_message()
        msg.num_positions = int(self.positions.size)
        msg.positions = self.positions.astype(np.float64).tolist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            positions=np.array(msg.positions, dtype=np.float64),
        )

    @classmethod
    @override
    def construct_default(cls, num_joints: int = 0) -> Self:
        return cls(header=TimestampHeader.construct_default(), positions=np.zeros(num_joints, dtype=np.float64))
