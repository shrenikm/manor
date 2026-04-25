"""
Simulation SensorBackend.

Reads sensor frames from a Drake simulation. The real implementation will
own references to a MultibodyPlant / SceneGraph and attached RgbdSensor
systems, advance them, and convert their output to RGB/Depth image
messages. For now this is a structural stub that returns empty frames --
enough to build and wire the Aegis diagram end-to-end.
"""

from __future__ import annotations

from typing import Any

import attr

from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class SimSensorBackendConfig:
    """
    Configuration for the simulation sensor backend.

    Kept deliberately sparse right now -- the concrete sim wiring (plant,
    scene graph, camera intrinsics) will land here as the system is built
    out.
    """

    rgb_height: int = 480
    rgb_width: int = 640
    depth_height: int = 480
    depth_width: int = 640


@attr.define
class SimSensorBackend:
    """
    SensorBackend that reads from a Drake simulation.

    The ``plant`` / ``scene_graph`` / ``context`` handles are accepted now so
    the integration seam exists, but they are not yet consumed by the stub
    implementations of ``read_rgb`` / ``read_depth``.
    """

    config: SimSensorBackendConfig = attr.field(factory=SimSensorBackendConfig)
    plant: Any = None
    scene_graph: Any = None
    plant_context: Any = None

    def read_rgb(self) -> RGBImageData:
        # TODO: render from an attached RgbdSensor and populate the bytes payload.
        image = RGBImageData.construct_default(height=self.config.rgb_height, width=self.config.rgb_width)
        return attr.evolve(image, header=TimestampHeader.from_system_time())

    def read_depth(self) -> DepthImageData:
        # TODO: render depth from an attached RgbdSensor and populate the bytes payload.
        image = DepthImageData.construct_default(height=self.config.depth_height, width=self.config.depth_width)
        return attr.evolve(image, header=TimestampHeader.from_system_time())
