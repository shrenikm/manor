"""
Full proprioception state.

Always carries joint state. Cartesian (tip pose / twist, bundled into
CartesianState) and ee (gripper positions / velocities, bundled into
EEState) sides are independently optional: skip cartesian_state when no
forward-kinematics tip frame is available, and skip ee_state on robots
without a gripper or when the gripper is not being read.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.cartesian_state import CartesianState
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.lcmtypes.lcmt_cartesian_state import lcmt_cartesian_state
from manor.common.definitions.lcmtypes.lcmt_ee_state import lcmt_ee_state
from manor.common.definitions.lcmtypes.lcmt_proprioception import lcmt_proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


@attr.frozen
class Proprioception(DefinitionBase):
    """
    Full proprioception state of a robot.
    """

    header: TimestampHeader
    joint_state: JointState
    cartesian_state: CartesianState | None = None
    ee_state: EEState | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("proprioception.capnp").VersionedProprioception

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_proprioception

    @override
    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init("header"))
        self.joint_state.to_versioned_capnp(builder.init("jointState"))

        if self.cartesian_state is None:
            builder.cartesianState.none = None
        else:
            self.cartesian_state.to_versioned_capnp(builder.cartesianState.init("some"))

        if self.ee_state is None:
            builder.eeState.none = None
        else:
            self.ee_state.to_versioned_capnp(builder.eeState.init("some"))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        cartesian_state = (
            CartesianState.from_versioned_capnp(reader.cartesianState.some)
            if reader.cartesianState.which() == "some"
            else None
        )
        ee_state = EEState.from_versioned_capnp(reader.eeState.some) if reader.eeState.which() == "some" else None
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            joint_state=JointState.from_versioned_capnp(reader.jointState),
            cartesian_state=cartesian_state,
            ee_state=ee_state,
        )

    @override
    def to_lcm_message(self) -> lcmt_proprioception:
        msg = lcmt_proprioception()
        msg.header = self.header.to_lcm_message()
        msg.joint_state = self.joint_state.to_lcm_message()

        msg.has_cartesian_state = self.cartesian_state is not None
        msg.cartesian_state = (
            self.cartesian_state.to_lcm_message() if self.cartesian_state is not None else lcmt_cartesian_state()
        )

        msg.has_ee_state = self.ee_state is not None
        msg.ee_state = self.ee_state.to_lcm_message() if self.ee_state is not None else lcmt_ee_state()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            joint_state=JointState.from_lcm_message(msg.joint_state),
            cartesian_state=(CartesianState.from_lcm_message(msg.cartesian_state) if msg.has_cartesian_state else None),
            ee_state=EEState.from_lcm_message(msg.ee_state) if msg.has_ee_state else None,
        )

    @classmethod
    @override
    def construct_default(cls, num_joints: int = 0, num_ee_dofs: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            joint_state=JointState.construct_default(num_joints=num_joints),
            cartesian_state=CartesianState.construct_default(),
            ee_state=EEState.construct_default(num_ee_dofs=num_ee_dofs),
        )
