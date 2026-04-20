"""
Full proprioception state.

Always carries joint state. End-effector state / pose / twist are optional:
e.g. if the robot has no EEF, or if forward kinematics was skipped upstream.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.lcmtypes.lcmt_eef_pose import lcmt_eef_pose
from manor.common.definitions.lcmtypes.lcmt_eef_state import lcmt_eef_state
from manor.common.definitions.lcmtypes.lcmt_eef_twist import lcmt_eef_twist
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
    eef_state: EEFState | None = None
    eef_pose: EEFPose | None = None
    eef_twist: EEFTwist | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("proprioception.capnp").VersionedProprioception

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_proprioception

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init("header"))
        self.joint_state.to_versioned_capnp(builder.init("jointState"))

        if self.eef_state is None:
            builder.eefState.none = None
        else:
            self.eef_state.to_versioned_capnp(builder.eefState.init("some"))

        if self.eef_pose is None:
            builder.eefPose.none = None
        else:
            self.eef_pose.to_versioned_capnp(builder.eefPose.init("some"))

        if self.eef_twist is None:
            builder.eefTwist.none = None
        else:
            self.eef_twist.to_versioned_capnp(builder.eefTwist.init("some"))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        eef_state = EEFState.from_versioned_capnp(reader.eefState.some) if reader.eefState.which() == "some" else None
        eef_pose = EEFPose.from_versioned_capnp(reader.eefPose.some) if reader.eefPose.which() == "some" else None
        eef_twist = EEFTwist.from_versioned_capnp(reader.eefTwist.some) if reader.eefTwist.which() == "some" else None
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            joint_state=JointState.from_versioned_capnp(reader.jointState),
            eef_state=eef_state,
            eef_pose=eef_pose,
            eef_twist=eef_twist,
        )

    @override
    def to_lcm_message(self) -> lcmt_proprioception:
        msg = lcmt_proprioception()
        msg.header = self.header.to_lcm_message()
        msg.joint_state = self.joint_state.to_lcm_message()

        msg.has_eef_state = self.eef_state is not None
        msg.eef_state = self.eef_state.to_lcm_message() if self.eef_state is not None else lcmt_eef_state()

        msg.has_eef_pose = self.eef_pose is not None
        msg.eef_pose = self.eef_pose.to_lcm_message() if self.eef_pose is not None else lcmt_eef_pose()

        msg.has_eef_twist = self.eef_twist is not None
        msg.eef_twist = self.eef_twist.to_lcm_message() if self.eef_twist is not None else lcmt_eef_twist()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            joint_state=JointState.from_lcm_message(msg.joint_state),
            eef_state=EEFState.from_lcm_message(msg.eef_state) if msg.has_eef_state else None,
            eef_pose=EEFPose.from_lcm_message(msg.eef_pose) if msg.has_eef_pose else None,
            eef_twist=EEFTwist.from_lcm_message(msg.eef_twist) if msg.has_eef_twist else None,
        )
