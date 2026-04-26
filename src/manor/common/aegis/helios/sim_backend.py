"""
Simulation SensorBackend.

Closes over a ``Sim`` instance and forwards camera reads to it. The Sim
owns the ``RgbdSensor`` instances (when wired in); this backend just
projects ``render_rgb`` / ``render_depth`` into the sensor backend
protocol Helios consumes.
"""

from __future__ import annotations

import attr

from manor.common.aegis.sim.sim import Sim
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData


@attr.frozen
class SimSensorBackendConfig:
    """
    Configuration for the simulation sensor backend.

    ``camera_id`` selects which camera registered on the Sim is read.
    The default ``"default"`` matches the Sim's stub camera.
    """

    camera_id: str = "default"


@attr.define
class SimSensorBackend:
    """
    SensorBackend that pulls frames from a shared ``Sim``.
    """

    sim: Sim
    config: SimSensorBackendConfig = attr.field(factory=SimSensorBackendConfig)

    def read_rgb(self) -> RGBImageData:
        return self.sim.render_rgb(camera_id=self.config.camera_id)

    def read_depth(self) -> DepthImageData:
        return self.sim.render_depth(camera_id=self.config.camera_id)
