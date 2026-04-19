"""
RGB image frame produced by a color camera.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.enums import ImageEncoding
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.rgb_image_data_t import rgb_image_data_t

_CAPNP = load_versioned_schema("rgb_image_data")

_ENCODING_TO_CAPNP: dict[ImageEncoding, str] = {
    ImageEncoding.RAW_RGB8: "rawRgb8",
    ImageEncoding.RAW_BGR8: "rawBgr8",
    ImageEncoding.JPEG: "jpeg",
    ImageEncoding.PNG: "png",
}
_CAPNP_TO_ENCODING: dict[str, ImageEncoding] = {v: k for k, v in _ENCODING_TO_CAPNP.items()}


@attr.frozen
class RGBImageData(DefinitionBase):
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
    LCM_CLASS: ClassVar[type] = rgb_image_data_t
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

    def to_lcm_message(self) -> rgb_image_data_t:
        msg = rgb_image_data_t()
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
