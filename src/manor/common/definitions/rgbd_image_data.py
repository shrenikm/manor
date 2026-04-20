"""
Paired RGB + depth frames.

Color and depth frames are independent -- they may have different dimensions,
different timestamps, and different encodings. This mirrors the RealSense /
Orbbec frameset model.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.lcmtypes.lcmt_rgbd_image_data import lcmt_rgbd_image_data
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    DEPTH = "depth"
    HEADER = "header"
    RGB = "rgb"


@attr.frozen
class RGBDImageData(DefinitionBase):
    """
    A color frame paired with a depth frame.
    """

    header: TimestampHeader
    rgb: RGBImageData
    depth: DepthImageData

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("rgbd_image_data.capnp").VersionedRgbdImageData

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_rgbd_image_data

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))
        self.rgb.to_versioned_capnp(builder.init(_CapnpField.RGB))
        self.depth.to_versioned_capnp(builder.init(_CapnpField.DEPTH))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            rgb=RGBImageData.from_versioned_capnp(reader.rgb),
            depth=DepthImageData.from_versioned_capnp(reader.depth),
        )

    @override
    def to_lcm_message(self) -> lcmt_rgbd_image_data:
        msg = lcmt_rgbd_image_data()
        msg.header = self.header.to_lcm_message()
        msg.rgb = self.rgb.to_lcm_message()
        msg.depth = self.depth.to_lcm_message()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            rgb=RGBImageData.from_lcm_message(msg.rgb),
            depth=DepthImageData.from_lcm_message(msg.depth),
        )
