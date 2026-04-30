"""
Observation input for policies.

Every field except the header is optional -- e.g. a policy-less control loop
may publish a bare observation with only a header, and sensor availability
varies across robots.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, ClassVar, Self, override

import attr

from manor.common.definitions.lcmtypes.lcmt_observation import lcmt_observation
from manor.common.definitions.lcmtypes.lcmt_proprioception import lcmt_proprioception
from manor.common.definitions.lcmtypes.lcmt_rgb_image_data import lcmt_rgb_image_data
from manor.common.definitions.lcmtypes.lcmt_rgbd_image_data import lcmt_rgbd_image_data
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.rgbd_image_data import RGBDImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, load_versioned_schema
from manor.common.definitions.utils.interfaces import DefinitionBase


class _CapnpField(StrEnum):
    HEADER = "header"


@attr.frozen
class Observation(DefinitionBase):
    """
    Policy observation. All sensor channels are optional.
    """

    header: TimestampHeader
    proprioception: Proprioception | None = None
    rgb_image: RGBImageData | None = None
    rgbd_image: RGBDImageData | None = None

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    @classmethod
    @override
    def get_capnp_schema(cls) -> CapnpStructSchema:
        return load_versioned_schema("observation.capnp").VersionedObservation

    @classmethod
    @override
    def get_lcm_class(cls) -> type:
        return lcmt_observation

    def to_capnp_current(self, builder: Any) -> None:
        self.header.to_versioned_capnp(builder.init(_CapnpField.HEADER))

        if self.proprioception is None:
            builder.proprioception.none = None
        else:
            self.proprioception.to_versioned_capnp(builder.proprioception.init("some"))

        if self.rgb_image is None:
            builder.rgbImage.none = None
        else:
            self.rgb_image.to_versioned_capnp(builder.rgbImage.init("some"))

        if self.rgbd_image is None:
            builder.rgbdImage.none = None
        else:
            self.rgbd_image.to_versioned_capnp(builder.rgbdImage.init("some"))

    @classmethod
    def from_capnp_v1(cls, reader: Any) -> Self:
        proprioception = (
            Proprioception.from_versioned_capnp(reader.proprioception.some)
            if reader.proprioception.which() == "some"
            else None
        )
        rgb_image = (
            RGBImageData.from_versioned_capnp(reader.rgbImage.some) if reader.rgbImage.which() == "some" else None
        )
        rgbd_image = (
            RGBDImageData.from_versioned_capnp(reader.rgbdImage.some) if reader.rgbdImage.which() == "some" else None
        )
        return cls(
            header=TimestampHeader.from_versioned_capnp(reader.header),
            proprioception=proprioception,
            rgb_image=rgb_image,
            rgbd_image=rgbd_image,
        )

    @override
    def to_lcm_message(self) -> lcmt_observation:
        msg = lcmt_observation()
        msg.header = self.header.to_lcm_message()

        msg.has_proprioception = self.proprioception is not None
        msg.proprioception = (
            self.proprioception.to_lcm_message() if self.proprioception is not None else lcmt_proprioception()
        )

        msg.has_rgb_image = self.rgb_image is not None
        msg.rgb_image = self.rgb_image.to_lcm_message() if self.rgb_image is not None else lcmt_rgb_image_data()

        msg.has_rgbd_image = self.rgbd_image is not None
        msg.rgbd_image = self.rgbd_image.to_lcm_message() if self.rgbd_image is not None else lcmt_rgbd_image_data()
        return msg

    @classmethod
    @override
    def from_lcm_message(cls, msg: Any) -> Self:
        return cls(
            header=TimestampHeader.from_lcm_message(msg.header),
            proprioception=Proprioception.from_lcm_message(msg.proprioception) if msg.has_proprioception else None,
            rgb_image=RGBImageData.from_lcm_message(msg.rgb_image) if msg.has_rgb_image else None,
            rgbd_image=RGBDImageData.from_lcm_message(msg.rgbd_image) if msg.has_rgbd_image else None,
        )

    @classmethod
    @override
    def construct_default(cls, num_joints: int = 0, num_ee_dofs: int = 0) -> Self:
        return cls(
            header=TimestampHeader.construct_default(),
            proprioception=Proprioception.construct_default(num_joints=num_joints, num_ee_dofs=num_ee_dofs),
            rgb_image=RGBImageData.construct_default(),
            rgbd_image=None,
        )
