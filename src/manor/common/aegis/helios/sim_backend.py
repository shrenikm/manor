"""
Simulation SensorBackend.

Closes over a ``Gaia`` instance and forwards camera reads to it. Gaia
owns the ``RgbdSensor`` instances (when wired in); this backend just
projects ``render_rgb`` / ``render_depth`` into the sensor backend
protocol Helios consumes.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self

import attr

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.exceptions import AegisConfigError


class _YamlKey(StrEnum):
    CAMERA_ID = "camera_id"


@attr.frozen
class SimSensorBackendConfig:
    """
    Configuration for the simulation sensor backend. ``camera_id``
    selects which camera registered on Gaia is read; the default
    ``"default"`` matches Gaia's stub camera.
    """

    camera_id: str = "default"

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        allowed = {key.value for key in _YamlKey}
        extras = set(d) - allowed
        if extras:
            raise AegisConfigError(
                f"sim_backend_config: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}"
            )
        camera_id = d.get(_YamlKey.CAMERA_ID, "default")
        if not isinstance(camera_id, str) or not camera_id:
            raise AegisConfigError(
                f"sim_backend_config.{_YamlKey.CAMERA_ID} must be a non-empty string; got {camera_id!r}"
            )
        return cls(camera_id=camera_id)


@attr.define
class SimSensorBackend:
    """
    SensorBackend that pulls frames from a shared ``Gaia``.
    """

    gaia: Gaia
    config: SimSensorBackendConfig = attr.field(factory=SimSensorBackendConfig)

    def read_rgb(self) -> RGBImageData:
        return self.gaia.render_rgb(camera_id=self.config.camera_id)

    def read_depth(self) -> DepthImageData:
        return self.gaia.render_depth(camera_id=self.config.camera_id)
