"""
Cartesian state of the end-effector tip frame at a single time instant.

Bundles the Cartesian pose and twist together so callers don't have to
correlate two parallel optionals on Proprioception. Both fields are
required at this layer; the whole state is what's optional on
Proprioception.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.lcmtypes.lcmt_cartesian_state import lcmt_cartesian_state
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    HEADER = "header"
    CARTESIAN_POSE = "cartesianPose"
    CARTESIAN_TWIST = "cartesianTwist"


@attr.frozen
class CartesianState(DefinitionBase):
    """
    Cartesian pose + twist of the end-effector tip at the same time instant.
    """

    header: TimestampHeader
    cartesian_pose: CartesianPose
    cartesian_twist: CartesianTwist

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("cartesian_state.capnp").VersionedCartesianState

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_cartesian_state

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        self.cartesian_pose.to_versioned_capnp(builder.init(_CapnpField.CARTESIAN_POSE))
        self.cartesian_twist.to_versioned_capnp(builder.init(_CapnpField.CARTESIAN_TWIST))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            cartesian_pose=CartesianPose.from_versioned_capnp(reader.cartesianPose),
            cartesian_twist=CartesianTwist.from_versioned_capnp(reader.cartesianTwist),
        )

    @override
    def to_lcm_message(self) -> lcmt_cartesian_state:
        msg = lcmt_cartesian_state()
        msg.header = self.header.to_lcm_message()
        msg.cartesian_pose = self.cartesian_pose.to_lcm_message()
        msg.cartesian_twist = self.cartesian_twist.to_lcm_message()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            cartesian_pose=CartesianPose.from_lcm_message(msg.cartesian_pose),
            cartesian_twist=CartesianTwist.from_lcm_message(msg.cartesian_twist),
        )

    @classmethod
    @override
    def construct_default(cls) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            cartesian_pose=CartesianPose.construct_default(),
            cartesian_twist=CartesianTwist.construct_default(),
        )
