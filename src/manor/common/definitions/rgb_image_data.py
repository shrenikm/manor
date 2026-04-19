"""
RGB image frame produced by a color camera.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions.lcmtypes.lcmt_rgb_image_data import lcmt_rgb_image_data
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import load_versioned_schema
from manor.common.definitions.utils.enums import ImageEncoding
from manor.common.definitions.utils.interfaces import IDefinition

_CAPNP = load_versioned_schema("rgb_image_data")

_ENCODING_TO_CAPNP: dict[ImageEncoding, str] = {
    ImageEncoding.RAW_RGB8: "rawRgb8",
    ImageEncoding.RAW_BGR8: "rawBgr8",
    ImageEncoding.JPEG: "jpeg",
    ImageEncoding.PNG: "png",
}
_CAPNP_TO_ENCODING: dict[str, ImageEncoding] = {v: k for k, v in _ENCODING_TO_CAPNP.items()}


@attr.frozen
class RGBImageData(IDefinition):
    """
    A single RGB frame. `data` is the raw pixel bytes when `encoding` is RAW_*,
    or a compressed image payload (e.g. JPEG/PNG bytes) otherwise.
    """

    header: TimestampHeader
    height: int
    width: int
    encoding: ImageEncoding
    data: bytes

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedRgbImageData
    LCM_CLASS: ClassVar[type] = lcmt_rgb_image_data
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        builder.height = int(self.height)
        builder.width = int(self.width)
        builder.encoding = _ENCODING_TO_CAPNP[self.encoding]
        builder.data = self.data

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            height=int(reader.height),
            width=int(reader.width),
            encoding=_CAPNP_TO_ENCODING[str(reader.encoding)],
            data=bytes(reader.data),
        )

    def to_lcm_message(self) -> lcmt_rgb_image_data:
        msg = lcmt_rgb_image_data()
        msg.header = self.header.to_lcm_message()
        msg.height = int(self.height)
        msg.width = int(self.width)
        msg.encoding = str(self.encoding.value)
        msg.num_bytes = len(self.data)
        msg.data = self.data
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            height=int(msg.height),
            width=int(msg.width),
            encoding=ImageEncoding(msg.encoding),
            data=bytes(msg.data),
        )
