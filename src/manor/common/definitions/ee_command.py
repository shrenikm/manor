"""
End-effector (gripper) command container.

Wraps an instantaneous gripper-side setpoint as a tagged-union of
either EEPositions (e.g. parallel-finger separation) or EEVelocities.
Exactly one variant must be set; construction with zero or more than
one raises InvalidDefinitionError.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, get_args, get_type_hints, override

import attr

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.lcmtypes.lcmt_ee_command import lcmt_ee_command
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.exceptions import InvalidDefinitionError


class _CapnpField(StrEnum):
    HEADER = "header"
    EE_POSITIONS = "eePositions"
    EE_VELOCITIES = "eeVelocities"


@attr.frozen
class EECommand(DefinitionBase):
    """
    An end-effector (gripper) command. Exactly one of the variant fields must be non-None.
    """

    header: TimestampHeader
    ee_positions: EEPositions | None = None
    ee_velocities: EEVelocities | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("ee_command.capnp").VersionedEECommand

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_ee_command

    @classmethod
    def _variant_fields(cls) -> list[attr.Attribute]:
        return [f for f in attr.fields(cls) if f.name != _CapnpField.HEADER.name.lower()]

    def _active_variant_field(self) -> attr.Attribute:
        for f in self._variant_fields():
            if getattr(self, f.name) is not None:
                return f
        raise InvalidDefinitionError("EECommand has no active variant")

    @classmethod
    def get_capnp_arm(cls, field: attr.Attribute) -> _CapnpField:
        return _CapnpField[field.name.upper()]

    def get_lcm_variant(self) -> int:
        active = self._active_variant_field()
        return self._variant_fields().index(active)

    @classmethod
    def get_lcm_type_from_variant(cls, tag: int) -> type:
        field = cls._variant_fields()[tag]
        hint = get_type_hints(cls)[field.name]
        return next(a for a in get_args(hint) if a is not type(None))

    def __attrs_post_init__(self) -> None:
        active = [f for f in self._variant_fields() if getattr(self, f.name) is not None]
        if len(active) != 1:
            raise InvalidDefinitionError(
                f"EECommand requires exactly one variant field to be set; got {len(active)}: {[f.name for f in active]}"
            )

    @override
    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        active = self._active_variant_field()
        arm = self.get_capnp_arm(active)
        getattr(self, active.name).to_versioned_capnp(builder.init(arm))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        arm = reader.which()
        try:
            capnp_field = _CapnpField(arm)
        except ValueError as e:
            raise InvalidDefinitionError(f"Unknown ee_command capnp arm {arm!r}") from e
        attr_field_name = capnp_field.name.lower()
        hint = get_type_hints(cls)[attr_field_name]
        variant_cls = next(a for a in get_args(hint) if a is not type(None))
        value = variant_cls.from_versioned_capnp(getattr(reader, arm))
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            **{attr_field_name: value},
        )

    @override
    def to_lcm_message(self) -> lcmt_ee_command:
        msg = lcmt_ee_command()
        msg.header = self.header.to_lcm_message()
        msg.variant = self.get_lcm_variant()
        active = self._active_variant_field()
        setattr(msg, active.name, getattr(self, active.name).to_lcm_message())
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        variant = int(msg.variant)
        variant_fields = cls._variant_fields()
        if not (0 <= variant < len(variant_fields)):
            raise InvalidDefinitionError(f"Unknown ee_command variant tag {variant}")
        field = variant_fields[variant]
        variant_cls = cls.get_lcm_type_from_variant(variant)
        value = variant_cls.from_lcm_message(getattr(msg, field.name))
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            **{field.name: value},
        )

    @classmethod
    @override
    def construct_default(cls, num_ee_dofs: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            ee_positions=EEPositions.construct_default(num_ee_dofs=num_ee_dofs),
        )
