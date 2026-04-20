"""
Timestamp header carried by every streaming definition.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.lcmtypes.lcmt_timestamp_header import lcmt_timestamp_header
from manor.common.definitions.utils.capnp_utils import load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


@attr.frozen
class TimestampHeader(DefinitionBase):
    """
    Monotonic and system-wall-clock timestamps, both in nanoseconds.
    """

    monotonic_ns: int
    system_ns: int

    LCM_CLASS: ClassVar[type] = lcmt_timestamp_header
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> Any:
        return load_versioned_schema("timestamp_header.capnp").VersionedTimestampHeader

    def _to_capnp_current(self, builder: Any) -> None:
        builder.monotonicNs = int(self.monotonic_ns)
        builder.systemNs = int(self.system_ns)

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
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
