"""
Policy action.

Metis emits an Action that drives Talos by way of Kyber. An Action carries
two independent groups of fields:

- Group 1 (arm): exactly one of joint_command, joint_trajectory_command,
  cartesian_command, cartesian_trajectory_command. The arm always needs a
  setpoint, but the policy chooses joint vs Cartesian and instantaneous
  vs trajectory.
- Group 2 (ee): at most one of ee_command, ee_trajectory_command. Gripper
  control is optional; many policies leave it untouched, so the action
  may carry neither field.

Constraints are enforced in __attrs_post_init__; violations raise
InvalidDefinitionError.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, get_args, get_type_hints, override

import attr

from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.cartesian_trajectory_command import CartesianTrajectoryCommand
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_trajectory_command import EETrajectoryCommand
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_trajectory_command import JointTrajectoryCommand
from manor.common.definitions.lcmtypes.lcmt_action import lcmt_action
from manor.common.definitions.lcmtypes.lcmt_cartesian_command import lcmt_cartesian_command
from manor.common.definitions.lcmtypes.lcmt_cartesian_trajectory_command import lcmt_cartesian_trajectory_command
from manor.common.definitions.lcmtypes.lcmt_ee_command import lcmt_ee_command
from manor.common.definitions.lcmtypes.lcmt_ee_trajectory_command import lcmt_ee_trajectory_command
from manor.common.definitions.lcmtypes.lcmt_joint_command import lcmt_joint_command
from manor.common.definitions.lcmtypes.lcmt_joint_trajectory_command import lcmt_joint_trajectory_command
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.exceptions import InvalidDefinitionError


class _ArmCapnpArm(StrEnum):
    JOINT_COMMAND = "jointCommand"
    JOINT_TRAJECTORY_COMMAND = "jointTrajectoryCommand"
    CARTESIAN_COMMAND = "cartesianCommand"
    CARTESIAN_TRAJECTORY_COMMAND = "cartesianTrajectoryCommand"


class _EECapnpArm(StrEnum):
    NONE = "none"
    EE_COMMAND = "eeCommand"
    EE_TRAJECTORY_COMMAND = "eeTrajectoryCommand"


_ARM_FIELDS: tuple[str, ...] = (
    "joint_command",
    "joint_trajectory_command",
    "cartesian_command",
    "cartesian_trajectory_command",
)
_EE_FIELDS: tuple[str, ...] = ("ee_command", "ee_trajectory_command")
_EE_NONE_TAG: int = -1


@attr.frozen
class Action(DefinitionBase):
    """
    A policy action. Group-1 arm fields require exactly one set; group-2 ee
    fields allow at most one.
    """

    header: TimestampHeader
    joint_command: JointCommand | None = None
    joint_trajectory_command: JointTrajectoryCommand | None = None
    cartesian_command: CartesianCommand | None = None
    cartesian_trajectory_command: CartesianTrajectoryCommand | None = None
    ee_command: EECommand | None = None
    ee_trajectory_command: EETrajectoryCommand | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    def __attrs_post_init__(self) -> None:
        active_arm = [name for name in _ARM_FIELDS if getattr(self, name) is not None]
        if len(active_arm) != 1:
            raise InvalidDefinitionError(
                f"Action requires exactly one arm command ({list(_ARM_FIELDS)}); got {len(active_arm)}: {active_arm}"
            )
        active_ee = [name for name in _EE_FIELDS if getattr(self, name) is not None]
        if len(active_ee) > 1:
            raise InvalidDefinitionError(
                f"Action allows at most one ee command ({list(_EE_FIELDS)}); got {len(active_ee)}: {active_ee}"
            )

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("action.capnp").VersionedAction

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_action

    def _active_arm_field(self) -> str:
        for name in _ARM_FIELDS:
            if getattr(self, name) is not None:
                return name
        raise InvalidDefinitionError("Action has no active arm command")

    def _active_ee_field(self) -> str | None:
        for name in _EE_FIELDS:
            if getattr(self, name) is not None:
                return name
        return None

    @override
    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init("header"))
        arm_field = self._active_arm_field()
        arm_arm = _ArmCapnpArm[arm_field.upper()]
        getattr(self, arm_field).to_versioned_capnp(builder.arm.init(arm_arm))
        ee_field = self._active_ee_field()
        if ee_field is None:
            builder.ee.none = None
        else:
            ee_arm = _EECapnpArm[ee_field.upper()]
            getattr(self, ee_field).to_versioned_capnp(builder.ee.init(ee_arm))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        arm_arm = reader.arm.which()
        try:
            arm_capnp = _ArmCapnpArm(arm_arm)
        except ValueError as e:
            raise InvalidDefinitionError(f"Unknown action arm capnp arm {arm_arm!r}") from e
        arm_attr = arm_capnp.name.lower()
        arm_hint = get_type_hints(cls)[arm_attr]
        arm_cls = next(a for a in get_args(arm_hint) if a is not type(None))
        arm_value = arm_cls.from_versioned_capnp(getattr(reader.arm, arm_arm))

        ee_arm = reader.ee.which()
        ee_kwargs: dict[str, Any] = {}
        if ee_arm != _EECapnpArm.NONE:
            try:
                ee_capnp = _EECapnpArm(ee_arm)
            except ValueError as e:
                raise InvalidDefinitionError(f"Unknown action ee capnp arm {ee_arm!r}") from e
            ee_attr = ee_capnp.name.lower()
            ee_hint = get_type_hints(cls)[ee_attr]
            ee_cls = next(a for a in get_args(ee_hint) if a is not type(None))
            ee_kwargs[ee_attr] = ee_cls.from_versioned_capnp(getattr(reader.ee, ee_arm))

        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            **{arm_attr: arm_value},
            **ee_kwargs,
        )

    @override
    def to_lcm_message(self) -> lcmt_action:
        msg = lcmt_action()
        msg.header = self.header.to_lcm_message()
        arm_field = self._active_arm_field()
        msg.arm_variant = _ARM_FIELDS.index(arm_field)
        setattr(msg, arm_field, getattr(self, arm_field).to_lcm_message())
        # Pad inactive arm fields with empty default messages.
        for name, lcm_cls in (
            ("joint_command", lcmt_joint_command),
            ("joint_trajectory_command", lcmt_joint_trajectory_command),
            ("cartesian_command", lcmt_cartesian_command),
            ("cartesian_trajectory_command", lcmt_cartesian_trajectory_command),
        ):
            if name != arm_field:
                setattr(msg, name, lcm_cls())
        ee_field = self._active_ee_field()
        if ee_field is None:
            msg.ee_variant = _EE_NONE_TAG
            msg.ee_command = lcmt_ee_command()
            msg.ee_trajectory_command = lcmt_ee_trajectory_command()
        else:
            msg.ee_variant = _EE_FIELDS.index(ee_field)
            setattr(msg, ee_field, getattr(self, ee_field).to_lcm_message())
            for name, lcm_cls in (
                ("ee_command", lcmt_ee_command),
                ("ee_trajectory_command", lcmt_ee_trajectory_command),
            ):
                if name != ee_field:
                    setattr(msg, name, lcm_cls())
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        arm_variant = int(msg.arm_variant)
        if not (0 <= arm_variant < len(_ARM_FIELDS)):
            raise InvalidDefinitionError(f"Unknown action arm variant tag {arm_variant}")
        arm_field = _ARM_FIELDS[arm_variant]
        arm_hint = get_type_hints(cls)[arm_field]
        arm_cls = next(a for a in get_args(arm_hint) if a is not type(None))
        arm_value = arm_cls.from_lcm_message(getattr(msg, arm_field))

        ee_variant = int(msg.ee_variant)
        ee_kwargs: dict[str, Any] = {}
        if ee_variant != _EE_NONE_TAG:
            if not (0 <= ee_variant < len(_EE_FIELDS)):
                raise InvalidDefinitionError(f"Unknown action ee variant tag {ee_variant}")
            ee_field = _EE_FIELDS[ee_variant]
            ee_hint = get_type_hints(cls)[ee_field]
            ee_cls = next(a for a in get_args(ee_hint) if a is not type(None))
            ee_kwargs[ee_field] = ee_cls.from_lcm_message(getattr(msg, ee_field))

        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            **{arm_field: arm_value},
            **ee_kwargs,
        )

    @classmethod
    @override
    def construct_default(cls, num_joints: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            joint_command=JointCommand.construct_default(num_joints=num_joints),
        )
