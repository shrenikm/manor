"""
Timestamp header carried by every streaming definition.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.interfaces import DefinitionBase
from manor_lcm.timestamp_header_t import timestamp_header_t

_CAPNP = load_versioned_schema("timestamp_header")


@attr.frozen
class TimestampHeader(DefinitionBase):
    """
    Monotonic and system-wall-clock timestamps, both in nanoseconds.
    """

    monotonic_ns: int
    system_ns: int

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedTimestampHeader
    LCM_CLASS: ClassVar[type] = timestamp_header_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        builder.monotonicNs = int(self.monotonic_ns)
        builder.systemNs = int(self.system_ns)

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            monotonic_ns=int(reader.monotonicNs),
            system_ns=int(reader.systemNs),
        )

    def to_lcm_message(self) -> timestamp_header_t:
        msg = timestamp_header_t()
        msg.monotonic_ns = int(self.monotonic_ns)
        msg.system_ns = int(self.system_ns)
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            monotonic_ns=int(msg.monotonic_ns),
            system_ns=int(msg.system_ns),
        )
