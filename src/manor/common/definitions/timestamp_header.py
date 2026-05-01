"""
Timestamp header carried by every streaming definition.
"""

from __future__ import annotations

import time
from typing import Any, ClassVar, Self, override

import attr

from manor.common.attrs_utils import is_non_negative_int
from manor.common.definitions.lcmtypes.lcmt_timestamp_header import lcmt_timestamp_header
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


@attr.frozen
class TimestampHeader(DefinitionBase):
    """
    Monotonic and system-wall-clock timestamps, both in nanoseconds.
    """

    monotonic_ns: int = attr.field(validator=is_non_negative_int())
    system_ns: int = attr.field(validator=is_non_negative_int())

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("timestamp_header.capnp").VersionedTimestampHeader

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_timestamp_header

    def to_capnp_current(self, builder: Any) -> None:
        builder.monotonicNs = int(self.monotonic_ns)
        builder.systemNs = int(self.system_ns)

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            monotonic_ns=int(reader.monotonicNs),
            system_ns=int(reader.systemNs),
        )

    @override
    def to_lcm_message(self) -> lcmt_timestamp_header:
        msg = lcmt_timestamp_header()
        msg.monotonic_ns = int(self.monotonic_ns)
        msg.system_ns = int(self.system_ns)
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            monotonic_ns=int(msg.monotonic_ns),
            system_ns=int(msg.system_ns),
        )

    @classmethod
    @override
    def construct_default(cls) -> Self:
        return cls(monotonic_ns=0, system_ns=0)

    @classmethod
    def from_system_time(cls) -> Self:
        """
        Build a header stamped with the current monotonic and system
        times in nanoseconds.
        """
        return cls(monotonic_ns=time.monotonic_ns(), system_ns=time.time_ns())
