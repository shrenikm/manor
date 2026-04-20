"""
Depth image frame produced by a depth / RGBD sensor.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.lcmtypes.lcmt_depth_image_data import lcmt_depth_image_data
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import load_versioned_schema
from manor.common.definitions.utils.enums import DepthEncoding
from manor.common.definitions.utils.interfaces import DefinitionBase

_ENCODING_TO_CAPNP: dict[DepthEncoding, str] = {
    DepthEncoding.RAW_FLOAT32_M: "rawFloat32M",
    DepthEncoding.RAW_UINT16_MM: "rawUint16Mm",
    DepthEncoding.PNG_UINT16_MM: "pngUint16Mm",
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

    LCM_CLASS: ClassVar[type] = lcmt_depth_image_data
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> Any:
        return load_versioned_schema("depth_image_data.capnp").VersionedDepthImageData

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

    @override
    def to_lcm_message(self) -> lcmt_depth_image_data:
        msg = lcmt_depth_image_data()
        msg.header = self.header.to_lcm_message()
        msg.height = int(self.height)
        msg.width = int(self.width)
        msg.encoding = str(self.encoding.value)
        msg.depth_scale = float(self.depth_scale)
        msg.num_bytes = len(self.data)
        msg.data = self.data
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            height=int(msg.height),
            width=int(msg.width),
            encoding=DepthEncoding(msg.encoding),
            data=bytes(msg.data),
            depth_scale=float(msg.depth_scale),
        )
