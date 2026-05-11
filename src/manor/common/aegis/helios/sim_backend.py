"""
Simulation SensorBackend.

Closes over a Gaia instance and forwards camera reads to it. Gaia owns the RgbdSensor instances (when wired in); this
backend just projects render_rgb / render_depth into the sensor backend protocol Helios consumes.
"""

from __future__ import annotations

from typing import Self

import attr

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData


@attr.frozen
class SimSensorBackendConfig:
    """
    Configuration for the simulation sensor backend. camera_id selects which camera registered on Gaia is read; the
    default "default" matches Gaia's stub camera.
    """

    camera_id: str = "default"

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "sim_backend_config"))


@attr.define
class SimSensorBackend:
    """
    SensorBackend that pulls frames from a shared Gaia.
    """

    gaia: Gaia
    config: SimSensorBackendConfig = attr.field(factory=SimSensorBackendConfig)

    def read_rgb(self) -> RGBImageData:
        return self.gaia.render_rgb(camera_id=self.config.camera_id)

    def read_depth(self) -> DepthImageData:
        return self.gaia.render_depth(camera_id=self.config.camera_id)
