"""
Full proprioception state.

Always carries joint state. End-effector state / pose / twist are optional:
e.g. if the robot has no EEF, or if forward kinematics was skipped upstream.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

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
from manor.common.definitions.utils.capnp_utils import load_versioned_schema
from manor.common.definitions.utils.interfaces import IDefinition

_CAPNP = load_versioned_schema("proprioception")


@attr.frozen
class Proprioception(IDefinition):
    """
    Full proprioception state of a robot.
    """

    header: TimestampHeader
    joint_state: JointState
    eef_state: EEFState | None = None
    eef_pose: EEFPose | None = None
    eef_twist: EEFTwist | None = None

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedProprioception
    LCM_CLASS: ClassVar[type] = lcmt_proprioception
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        self.joint_state._to_capnp_current(builder.init("jointState"))

        if self.eef_state is None:
            builder.eefState.none = None
        else:
            self.eef_state._to_capnp_current(builder.eefState.init("some"))

        if self.eef_pose is None:
            builder.eefPose.none = None
        else:
            self.eef_pose._to_capnp_current(builder.eefPose.init("some"))

        if self.eef_twist is None:
            builder.eefTwist.none = None
        else:
            self.eef_twist._to_capnp_current(builder.eefTwist.init("some"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        eef_state = (
            EEFState._from_capnp_v1(reader.eefState.some)
            if reader.eefState.which() == "some"
            else None
        )
        eef_pose = (
            EEFPose._from_capnp_v1(reader.eefPose.some)
            if reader.eefPose.which() == "some"
            else None
        )
        eef_twist = (
            EEFTwist._from_capnp_v1(reader.eefTwist.some)
            if reader.eefTwist.which() == "some"
            else None
        )
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            joint_state=JointState._from_capnp_v1(reader.jointState),
            eef_state=eef_state,
            eef_pose=eef_pose,
            eef_twist=eef_twist,
        )

    def to_lcm_message(self) -> lcmt_proprioception:
        msg = lcmt_proprioception()
        msg.header = self.header.to_lcm_message()
        msg.joint_state = self.joint_state.to_lcm_message()

        msg.has_eef_state = 1 if self.eef_state is not None else 0
        msg.eef_state = (
            self.eef_state.to_lcm_message() if self.eef_state is not None else lcmt_eef_state()
        )

        msg.has_eef_pose = 1 if self.eef_pose is not None else 0
        msg.eef_pose = (
            self.eef_pose.to_lcm_message() if self.eef_pose is not None else lcmt_eef_pose()
        )

        msg.has_eef_twist = 1 if self.eef_twist is not None else 0
        msg.eef_twist = (
            self.eef_twist.to_lcm_message() if self.eef_twist is not None else lcmt_eef_twist()
        )
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            joint_state=JointState.from_lcm_message(msg.joint_state),
            eef_state=EEFState.from_lcm_message(msg.eef_state) if msg.has_eef_state else None,
            eef_pose=EEFPose.from_lcm_message(msg.eef_pose) if msg.has_eef_pose else None,
            eef_twist=EEFTwist.from_lcm_message(msg.eef_twist) if msg.has_eef_twist else None,
        )
