"""
Paired RGB + depth frames.

Color and depth frames are independent -- they may have different dimensions,
different timestamps, and different encodings. This mirrors the RealSense /
Orbbec frameset model.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.rgbd_image_data_t import rgbd_image_data_t

_CAPNP = load_versioned_schema("rgbd_image_data")


@attr.frozen
class RGBDImageData(DefinitionBase):
    """
    A color frame paired with a depth frame.
    """

    header: TimestampHeader
    rgb: RGBImageData
    depth: DepthImageData

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedRgbdImageData
    LCM_CLASS: ClassVar[type] = rgbd_image_data_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))
        self.rgb._to_capnp_current(builder.init("rgb"))
        self.depth._to_capnp_current(builder.init("depth"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            rgb=RGBImageData._from_capnp_v1(reader.rgb),
            depth=DepthImageData._from_capnp_v1(reader.depth),
        )

    def to_lcm_message(self) -> rgbd_image_data_t:
        msg = rgbd_image_data_t()
        msg.header = self.header.to_lcm_message()
        msg.rgb = self.rgb.to_lcm_message()
        msg.depth = self.depth.to_lcm_message()
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            rgb=RGBImageData.from_lcm_message(msg.rgb),
            depth=DepthImageData.from_lcm_message(msg.depth),
        )
