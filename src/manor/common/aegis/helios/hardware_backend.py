"""
Hardware SensorBackend.

Wraps the real camera SDK (e.g. RealSense / Orbbec). For now the SDK calls
are mocked so the backend can be constructed and exercised end-to-end;
``read_rgb`` / ``read_depth`` return empty frames with a fresh system-time
header.
"""

from __future__ import annotations

import attr

from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class HardwareSensorBackendConfig:
    """
    Connection info for the real sensor. Populated fields will grow as the
    hardware driver is swapped in.
    """

    serial_number: str = ""
    rgb_height: int = 480
    rgb_width: int = 640
    depth_height: int = 480
    depth_width: int = 640


@attr.define
class HardwareSensorBackend:
    """
    SensorBackend that reads from a real camera. SDK calls are currently
    stubbed; the ``_device`` handle represents the future SDK object.
    """

    config: HardwareSensorBackendConfig = attr.field(factory=HardwareSensorBackendConfig)
    _device: object | None = attr.field(default=None, init=False)

    def start(self) -> None:
        # TODO: open the SDK stream(s).
        self._device = object()

    def stop(self) -> None:
        # TODO: close the SDK stream(s) and release resources.
        self._device = None

    def read_rgb(self) -> RGBImageData:
        # TODO: pull the latest color frame from the SDK.
        image = RGBImageData.construct_default(height=self.config.rgb_height, width=self.config.rgb_width)
        return attr.evolve(image, header=TimestampHeader.from_system_time())

    def read_depth(self) -> DepthImageData:
        # TODO: pull the latest depth frame from the SDK.
        image = DepthImageData.construct_default(height=self.config.depth_height, width=self.config.depth_width)
        return attr.evolve(image, header=TimestampHeader.from_system_time())
