"""
Policy action.

An Action is exactly one of the variant fields. Policies can emit joint-space
or EEF-space commands, either as a single setpoint or as a full trajectory.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_pose_trajectory import EEFPoseTrajectory
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.eef_twist_trajectory import EEFTwistTrajectory
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_positions_trajectory import JointPositionsTrajectory
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.joint_velocities_trajectory import JointVelocitiesTrajectory
from manor.common.definitions.lcmtypes.lcmt_action import lcmt_action
from manor.common.definitions.lcmtypes.lcmt_eef_pose import lcmt_eef_pose
from manor.common.definitions.lcmtypes.lcmt_eef_pose_trajectory import lcmt_eef_pose_trajectory
from manor.common.definitions.lcmtypes.lcmt_eef_twist import lcmt_eef_twist
from manor.common.definitions.lcmtypes.lcmt_eef_twist_trajectory import lcmt_eef_twist_trajectory
from manor.common.definitions.lcmtypes.lcmt_joint_positions import lcmt_joint_positions
from manor.common.definitions.lcmtypes.lcmt_joint_positions_trajectory import (
    lcmt_joint_positions_trajectory,
)
from manor.common.definitions.lcmtypes.lcmt_joint_velocities import lcmt_joint_velocities
from manor.common.definitions.lcmtypes.lcmt_joint_velocities_trajectory import (
    lcmt_joint_velocities_trajectory,
)
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.exceptions import InvalidDefinitionError

# Field name -> (capnp union arm, LCM variant tag)
_VARIANTS: tuple[tuple[str, str, int], ...] = (
    ("joint_positions", "jointPositions", 0),
    ("joint_positions_trajectory", "jointPositionsTrajectory", 1),
    ("joint_velocities", "jointVelocities", 2),
    ("joint_velocities_trajectory", "jointVelocitiesTrajectory", 3),
    ("eef_pose", "eefPose", 4),
    ("eef_pose_trajectory", "eefPoseTrajectory", 5),
    ("eef_twist", "eefTwist", 6),
    ("eef_twist_trajectory", "eefTwistTrajectory", 7),
)
_FIELD_TO_CAPNP_ARM = {name: arm for name, arm, _ in _VARIANTS}
_CAPNP_ARM_TO_FIELD = {arm: name for name, arm, _ in _VARIANTS}
_FIELD_TO_VARIANT = {name: tag for name, _, tag in _VARIANTS}
_VARIANT_TO_FIELD = {tag: name for name, _, tag in _VARIANTS}
_FIELD_TO_CLASS: dict[str, type] = {
    "joint_positions": JointPositions,
    "joint_positions_trajectory": JointPositionsTrajectory,
    "joint_velocities": JointVelocities,
    "joint_velocities_trajectory": JointVelocitiesTrajectory,
    "eef_pose": EEFPose,
    "eef_pose_trajectory": EEFPoseTrajectory,
    "eef_twist": EEFTwist,
    "eef_twist_trajectory": EEFTwistTrajectory,
}
_LCM_DEFAULTS: dict[str, type] = {
    "joint_positions": lcmt_joint_positions,
    "joint_positions_trajectory": lcmt_joint_positions_trajectory,
    "joint_velocities": lcmt_joint_velocities,
    "joint_velocities_trajectory": lcmt_joint_velocities_trajectory,
    "eef_pose": lcmt_eef_pose,
    "eef_pose_trajectory": lcmt_eef_pose_trajectory,
    "eef_twist": lcmt_eef_twist,
    "eef_twist_trajectory": lcmt_eef_twist_trajectory,
}


@attr.frozen
class Action(DefinitionBase):
    """
    A policy action. Exactly one of the variant fields must be non-None.
    """

    header: TimestampHeader
    joint_positions: JointPositions | None = None
    joint_positions_trajectory: JointPositionsTrajectory | None = None
    joint_velocities: JointVelocities | None = None
    joint_velocities_trajectory: JointVelocitiesTrajectory | None = None
    eef_pose: EEFPose | None = None
    eef_pose_trajectory: EEFPoseTrajectory | None = None
    eef_twist: EEFTwist | None = None
    eef_twist_trajectory: EEFTwistTrajectory | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("action.capnp").VersionedAction

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_action

    def __attrs_post_init__(self) -> None:
        active = [name for name, _, _ in _VARIANTS if getattr(self, name) is not None]
        if len(active) != 1:
            raise InvalidDefinitionError(
                f"Action requires exactly one variant field to be set; got {len(active)}: {active}"
            )

    def _active_field(self) -> str:
        for name, _, _ in _VARIANTS:
            if getattr(self, name) is not None:
                return name
        raise InvalidDefinitionError("Action has no active variant")

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init("header"))
        field = self._active_field()
        arm = _FIELD_TO_CAPNP_ARM[field]
        value = getattr(self, field)
        value.to_versioned_capnp(builder.init(arm))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        arm = reader.which()
        if arm not in _CAPNP_ARM_TO_FIELD:
            raise InvalidDefinitionError(f"Unknown action capnp arm {arm!r}")
        field = _CAPNP_ARM_TO_FIELD[arm]
        value = _FIELD_TO_CLASS[field].from_versioned_capnp(getattr(reader, arm))
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            **{field: value},
        )

    @override
    def to_lcm_message(self) -> lcmt_action:
        msg = lcmt_action()
        msg.header = self.header.to_lcm_message()
        field = self._active_field()
        msg.variant = _FIELD_TO_VARIANT[field]
        for name, _, _ in _VARIANTS:
            value = getattr(self, name)
            if value is not None:
                setattr(msg, name, value.to_lcm_message())
            else:
                setattr(msg, name, _LCM_DEFAULTS[name]())
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        variant = int(msg.variant)
        if variant not in _VARIANT_TO_FIELD:
            raise InvalidDefinitionError(f"Unknown action variant tag {variant}")
        field = _VARIANT_TO_FIELD[variant]
        value = _FIELD_TO_CLASS[field].from_lcm_message(getattr(msg, field))
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            **{field: value},
        )
