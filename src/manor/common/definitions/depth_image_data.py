"""
Depth image frame produced by a depth / RGBD sensor.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.enums import DepthEncoding
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.depth_image_data_t import depth_image_data_t

_CAPNP = load_versioned_schema("depth_image_data")

_ENCODING_TO_CAPNP: dict[DepthEncoding, str] = {
    DepthEncoding.RAW_FLOAT32_M: "rawFloat32M",
    DepthEncoding.RAW_UINT16_MM: "rawUint16Mm",
    DepthEncoding.PNG_UINT16: "pngUint16",
}
_CAPNP_TO_ENCODING: dict[str, DepthEncoding] = {v: k for k, v in _ENCODING_TO_CAPNP.items()}


@attr.frozen
class DepthImageData(DefinitionBase):
    """
    A single depth frame. `data` is interpreted per `encoding`, and multiplied
    by `depth_scale` to get meters.
    """

    header: TimestampHeader
    height: int
    width: int
    encoding: DepthEncoding
    data: bytes
    depth_scale: float

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedDepthImageData
    LCM_CLASS: ClassVar[type] = depth_image_data_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        builder.height = int(self.height)
        builder.width = int(self.width)
        builder.encoding = _ENCODING_TO_CAPNP[self.encoding]
        builder.data = self.data
        builder.depthScale = float(self.depth_scale)

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            height=int(reader.height),
            width=int(reader.width),
            encoding=_CAPNP_TO_ENCODING[str(reader.encoding)],
            data=bytes(reader.data),
            depth_scale=float(reader.depthScale),
        )

    def to_lcm_message(self) -> depth_image_data_t:
        msg = depth_image_data_t()
        msg.header = self.header.to_lcm_message()
        msg.height = int(self.height)
        msg.width = int(self.width)
        msg.encoding = str(self.encoding.value)
        msg.depth_scale = float(self.depth_scale)
        msg.num_bytes = len(self.data)
        msg.data = self.data
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            height=int(msg.height),
            width=int(msg.width),
            encoding=DepthEncoding(msg.encoding),
            data=bytes(msg.data),
            depth_scale=float(msg.depth_scale),
        )
