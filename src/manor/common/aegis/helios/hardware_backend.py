"""
Hardware SensorBackend.

Wraps the real camera SDK (e.g. RealSense / Orbbec). For now the SDK calls
are mocked so the backend can be constructed and exercised end-to-end;
``read_rgb`` / ``read_depth`` return empty frames with a fresh system-time
header.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self

import attr

from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError


class _YamlKey(StrEnum):
    SERIAL_NUMBER = "serial_number"
    RGB_HEIGHT = "rgb_height"
    RGB_WIDTH = "rgb_width"
    DEPTH_HEIGHT = "depth_height"
    DEPTH_WIDTH = "depth_width"


def _require_int(value: object, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise AegisConfigError(f"'{field_name}' must be an int; got {type(value).__name__}")
    return value


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

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        allowed = {key.value for key in _YamlKey}
        extras = set(d) - allowed
        if extras:
            raise AegisConfigError(
                f"hardware_backend_config: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}"
            )
        serial_number = d.get(_YamlKey.SERIAL_NUMBER, "")
        if not isinstance(serial_number, str):
            raise AegisConfigError(
                f"hardware_backend_config.{_YamlKey.SERIAL_NUMBER} must be a string; got {type(serial_number).__name__}"
            )
        return cls(
            serial_number=serial_number,
            rgb_height=_require_int(
                d.get(_YamlKey.RGB_HEIGHT, 480),
                f"hardware_backend_config.{_YamlKey.RGB_HEIGHT}",
            ),
            rgb_width=_require_int(
                d.get(_YamlKey.RGB_WIDTH, 640),
                f"hardware_backend_config.{_YamlKey.RGB_WIDTH}",
            ),
            depth_height=_require_int(
                d.get(_YamlKey.DEPTH_HEIGHT, 480),
                f"hardware_backend_config.{_YamlKey.DEPTH_HEIGHT}",
            ),
            depth_width=_require_int(
                d.get(_YamlKey.DEPTH_WIDTH, 640),
                f"hardware_backend_config.{_YamlKey.DEPTH_WIDTH}",
            ),
        )


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
