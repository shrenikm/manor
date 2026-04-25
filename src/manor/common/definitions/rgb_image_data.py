"""
RGB image frame produced by a color camera.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.lcmtypes.lcmt_rgb_image_data import lcmt_rgb_image_data
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.enums import ImageEncoding
from manor.common.definitions.utils.interfaces import DefinitionBase

_ENCODING_TO_CAPNP: dict[ImageEncoding, str] = {
    ImageEncoding.RAW_RGB8: "rawRgb8",
    ImageEncoding.RAW_BGR8: "rawBgr8",
    ImageEncoding.JPEG: "jpeg",
    ImageEncoding.PNG: "png",
}
_CAPNP_TO_ENCODING: dict[str, ImageEncoding] = {v: k for k, v in _ENCODING_TO_CAPNP.items()}


class _CapnpField(StrEnum):
    HEADER = "header"


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

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("rgb_image_data.capnp").VersionedRgbImageData

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_rgb_image_data

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        builder.height = int(self.height)
        builder.width = int(self.width)
        builder.encoding = _ENCODING_TO_CAPNP[self.encoding]
        builder.data = self.data

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            height=int(reader.height),
            width=int(reader.width),
            encoding=_CAPNP_TO_ENCODING[str(reader.encoding)],
            data=bytes(reader.data),
        )

    @override
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
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            height=int(msg.height),
            width=int(msg.width),
            encoding=ImageEncoding(msg.encoding),
            data=bytes(msg.data),
        )

    @classmethod
    @override
    def construct_default(cls, height: int = 0, width: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            height=height,
            width=width,
            encoding=ImageEncoding.RAW_RGB8,
            data=b"",
        )
