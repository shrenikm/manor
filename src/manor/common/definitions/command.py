"""
Robot command.

The controller (Kyber) emits a Command to the hardware interface (Talos).
Exactly one of the variant fields must be non-None.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.lcmtypes.lcmt_command import lcmt_command
from manor.common.definitions.lcmtypes.lcmt_eef_pose import lcmt_eef_pose
from manor.common.definitions.lcmtypes.lcmt_eef_twist import lcmt_eef_twist
from manor.common.definitions.lcmtypes.lcmt_joint_positions import lcmt_joint_positions
from manor.common.definitions.lcmtypes.lcmt_joint_velocities import lcmt_joint_velocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.exceptions import InvalidDefinitionError

_VARIANTS: tuple[tuple[str, str, int], ...] = (
    ("joint_positions", "jointPositions", 0),
    ("joint_velocities", "jointVelocities", 1),
    ("eef_pose", "eefPose", 2),
    ("eef_twist", "eefTwist", 3),
)
_FIELD_TO_CAPNP_ARM = {name: arm for name, arm, _ in _VARIANTS}
_CAPNP_ARM_TO_FIELD = {arm: name for name, arm, _ in _VARIANTS}
_FIELD_TO_VARIANT = {name: tag for name, _, tag in _VARIANTS}
_VARIANT_TO_FIELD = {tag: name for name, _, tag in _VARIANTS}
_FIELD_TO_CLASS: dict[str, type] = {
    "joint_positions": JointPositions,
    "joint_velocities": JointVelocities,
    "eef_pose": EEFPose,
    "eef_twist": EEFTwist,
}
_LCM_DEFAULTS: dict[str, type] = {
    "joint_positions": lcmt_joint_positions,
    "joint_velocities": lcmt_joint_velocities,
    "eef_pose": lcmt_eef_pose,
    "eef_twist": lcmt_eef_twist,
}


@attr.frozen
class Command(DefinitionBase):
    """
    A hardware-facing command. Exactly one of the variant fields must be non-None.
    """

    header: TimestampHeader
    joint_positions: JointPositions | None = None
    joint_velocities: JointVelocities | None = None
    eef_pose: EEFPose | None = None
    eef_twist: EEFTwist | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("command.capnp").VersionedCommand

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_command

    def __attrs_post_init__(self) -> None:
        active = [name for name, _, _ in _VARIANTS if getattr(self, name) is not None]
        if len(active) != 1:
            raise InvalidDefinitionError(
                f"Command requires exactly one variant field to be set; got {len(active)}: {active}"
            )

    def _active_field(self) -> str:
        for name, _, _ in _VARIANTS:
            if getattr(self, name) is not None:
                return name
        raise InvalidDefinitionError("Command has no active variant")

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_versioned_capnp(builder.init("header"))
        field = self._active_field()
        arm = _FIELD_TO_CAPNP_ARM[field]
        value = getattr(self, field)
        value._to_versioned_capnp(builder.init(arm))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        arm = reader.which()
        if arm not in _CAPNP_ARM_TO_FIELD:
            raise InvalidDefinitionError(f"Unknown command capnp arm {arm!r}")
        field = _CAPNP_ARM_TO_FIELD[arm]
        value = _FIELD_TO_CLASS[field]._from_versioned_capnp(getattr(reader, arm))
        return cls(
            header=TimestampHeader._from_versioned_capnp(reader.header),
            **{field: value},
        )

    @override
    def to_lcm_message(self) -> lcmt_command:
        msg = lcmt_command()
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
            raise InvalidDefinitionError(f"Unknown command variant tag {variant}")
        field = _VARIANT_TO_FIELD[variant]
        value = _FIELD_TO_CLASS[field].from_lcm_message(getattr(msg, field))
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            **{field: value},
        )
