"""
Robot command.

The controller (Kyber) emits a Command to the hardware interface (Talos).
Exactly one of the variant fields must be non-None.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import InvalidDefinitionError
from manor_lcm.command_t import command_t
from manor_lcm.eef_pose_t import eef_pose_t
from manor_lcm.eef_twist_t import eef_twist_t
from manor_lcm.joint_positions_t import joint_positions_t
from manor_lcm.joint_velocities_t import joint_velocities_t

_CAPNP = load_versioned_schema("command")

_VARIANTS: tuple[tuple[str, str, int], ...] = (
    ("joint_positions",  "jointPositions",  0),
    ("joint_velocities", "jointVelocities", 1),
    ("eef_pose",         "eefPose",         2),
    ("eef_twist",        "eefTwist",        3),
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
    "joint_positions": joint_positions_t,
    "joint_velocities": joint_velocities_t,
    "eef_pose": eef_pose_t,
    "eef_twist": eef_twist_t,
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

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedCommand
    LCM_CLASS: ClassVar[type] = command_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

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
        self.header._to_capnp_current(builder.init("header"))
        field = self._active_field()
        arm = _FIELD_TO_CAPNP_ARM[field]
        value = getattr(self, field)
        value._to_capnp_current(builder.init(arm))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        arm = reader.which()
        if arm not in _CAPNP_ARM_TO_FIELD:
            raise InvalidDefinitionError(f"Unknown command capnp arm {arm!r}")
        field = _CAPNP_ARM_TO_FIELD[arm]
        value = _FIELD_TO_CLASS[field]._from_capnp_v1(getattr(reader, arm))
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            **{field: value},
        )

    def to_lcm_message(self) -> command_t:
        msg = command_t()
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
            raise InvalidDefinitionError(f"Unknown command variant tag {variant}")
        field = _VARIANT_TO_FIELD[variant]
        value = _FIELD_TO_CLASS[field].from_lcm_message(getattr(msg, field))
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            **{field: value},
        )
