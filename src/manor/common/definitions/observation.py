"""
Observation input for policies.

Every field except the header is optional -- e.g. a policy-less control loop
may publish a bare observation with only a header, and sensor availability
varies across robots.
"""

from __future__ import annotations

from typing import Any, ClassVar, Self

import attr

from manor.common.definitions._capnp_utils import load_versioned_schema
from manor.common.definitions.interfaces import DefinitionBase
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.rgbd_image_data import RGBDImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor_lcm.observation_t import observation_t
from manor_lcm.proprioception_t import proprioception_t
from manor_lcm.rgb_image_data_t import rgb_image_data_t
from manor_lcm.rgbd_image_data_t import rgbd_image_data_t

_CAPNP = load_versioned_schema("observation")


@attr.frozen
class Observation(DefinitionBase):
    """
    Policy observation. All sensor channels are optional.
    """

    header: TimestampHeader
    proprioception: Proprioception | None = None
    rgb_image: RGBImageData | None = None
    rgbd_image: RGBDImageData | None = None

    VERSION: ClassVar[str] = "1.0.0"
    CAPNP_SCHEMA: ClassVar[Any] = _CAPNP.VersionedObservation
    LCM_CLASS: ClassVar[type] = observation_t
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def _to_capnp_current(self, builder: Any) -> None:
        self.header._to_capnp_current(builder.init("header"))

        if self.proprioception is None:
            builder.proprioception.none = None
        else:
            self.proprioception._to_capnp_current(builder.proprioception.init("some"))

        if self.rgb_image is None:
            builder.rgbImage.none = None
        else:
            self.rgb_image._to_capnp_current(builder.rgbImage.init("some"))

        if self.rgbd_image is None:
            builder.rgbdImage.none = None
        else:
            self.rgbd_image._to_capnp_current(builder.rgbdImage.init("some"))

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        proprioception = (
            Proprioception._from_capnp_v1(reader.proprioception.some)
            if reader.proprioception.which() == "some"
            else None
        )
        rgb_image = (
            RGBImageData._from_capnp_v1(reader.rgbImage.some)
            if reader.rgbImage.which() == "some"
            else None
        )
        rgbd_image = (
            RGBDImageData._from_capnp_v1(reader.rgbdImage.some)
            if reader.rgbdImage.which() == "some"
            else None
        )
        return cls(
            header=TimestampHeader._from_capnp_v1(reader.header),
            proprioception=proprioception,
            rgb_image=rgb_image,
            rgbd_image=rgbd_image,
        )

    def to_lcm_message(self) -> observation_t:
        msg = observation_t()
        msg.header = self.header.to_lcm_message()

        msg.has_proprioception = 1 if self.proprioception is not None else 0
        msg.proprioception = (
            self.proprioception.to_lcm_message() if self.proprioception is not None else proprioception_t()
        )

        msg.has_rgb_image = 1 if self.rgb_image is not None else 0
        msg.rgb_image = (
            self.rgb_image.to_lcm_message() if self.rgb_image is not None else rgb_image_data_t()
        )

        msg.has_rgbd_image = 1 if self.rgbd_image is not None else 0
        msg.rgbd_image = (
            self.rgbd_image.to_lcm_message() if self.rgbd_image is not None else rgbd_image_data_t()
        )
        return msg

    @classmethod
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            proprioception=Proprioception.from_lcm_message(msg.proprioception)
            if msg.has_proprioception
            else None,
            rgb_image=RGBImageData.from_lcm_message(msg.rgb_image) if msg.has_rgb_image else None,
            rgbd_image=RGBDImageData.from_lcm_message(msg.rgbd_image)
            if msg.has_rgbd_image
            else None,
        )
