"""
Policy action.

An Action is exactly one of the variant fields. Policies can emit joint-space
or EEF-space commands, either as a single setpoint or as a full trajectory.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr
from manor_lcm.action_t import action_t
from manor_lcm.eef_pose_t import eef_pose_t
from manor_lcm.eef_pose_trajectory_t import eef_pose_trajectory_t
from manor_lcm.eef_twist_t import eef_twist_t
from manor_lcm.eef_twist_trajectory_t import eef_twist_trajectory_t
from manor_lcm.joint_positions_t import joint_positions_t
from manor_lcm.joint_positions_trajectory_t import joint_positions_trajectory_t
from manor_lcm.joint_velocities_t import joint_velocities_t
from manor_lcm.joint_velocities_trajectory_t import joint_velocities_trajectory_t

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_pose_trajectory import EEFPoseTrajectory
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.eef_twist_trajectory import EEFTwistTrajectory
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_positions_trajectory import JointPositionsTrajectory
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.joint_velocities_trajectory import JointVelocitiesTrajectory
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError

_CAPNP = load_versioned_schema("action")

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
    "joint_positions": joint_positions_t,
    "joint_positions_trajectory": joint_positions_trajectory_t,
    "joint_velocities": joint_velocities_t,
    "joint_velocities_trajectory": joint_velocities_trajectory_t,
    "eef_pose": eef_pose_t,
    "eef_pose_trajectory": eef_pose_trajectory_t,
    "eef_twist": eef_twist_t,
    "eef_twist_trajectory": eef_twist_trajectory_t,
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

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedAction
    LCM_CLASS: ClassVar[type] = action_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

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

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        field = self._active_field()
        arm = _FIELD_TO_CAPNP_ARM[field]
        value = getattr(self, field)
        value._to_capnp_current(builder.init(arm))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        arm = reader.which()
        if arm not in _CAPNP_ARM_TO_FIELD:
            raise InvalidDefinitionError(f"Unknown action capnp arm {arm!r}")
        field = _CAPNP_ARM_TO_FIELD[arm]
        value = _FIELD_TO_CLASS[field]._from_capnp_v1(getattr(reader, arm))
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            **{field: value},
        )

    def to_lcm_message(self) -> action_t:
        msg = action_t()
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
