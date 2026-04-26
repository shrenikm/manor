"""
Helios-process runner (hardware mode only).

In hardware mode helios is its own process: a single ``Helios``
LeafSystem driving a ``HardwareSensorBackend``, with LCM publishers
on the RGB / depth channels. There are no inputs.

In sim mode the helios role is fulfilled by the gylos process
(see ``run.gylos``), because ``SimSensorBackend`` closes over a
shared in-process ``Gaia`` handle that can't cross a process
boundary -- so a separate helios process in sim would be a
passthrough subscriber. Don't launch this runner in sim.

Standalone usage:

    python -m manor.common.aegis.run.helios < lite6_default.yaml.json
"""

from __future__ import annotations

import json
import sys

from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import AegisAdapterPorts, AegisLCMPublisherAdapter
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.aegis.helios.hardware_backend import HardwareSensorBackend
from manor.common.aegis.helios.helios import Helios, HeliosConfig, HeliosPorts
from manor.common.aegis.run.run_utils import advance_until_signal
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData


def run_helios(helios_config: HeliosConfig, lcm: DrakeLcm | None = None) -> None:
    """
    Build the helios-process diagram around a hardware sensor backend
    and advance forever (until SIGTERM / SIGINT).
    """
    lcm = lcm if lcm is not None else DrakeLcm()
    backend = HardwareSensorBackend(config=helios_config.hardware_backend_config)

    builder = DiagramBuilder()
    builder.AddSystem(LcmInterfaceSystem(lcm))

    helios = builder.AddSystem(
        Helios(
            backend=backend,
            publish_rgb_frequency_hz=helios_config.publish_rgb_frequency_hz,
            publish_depth_frequency_hz=helios_config.publish_depth_frequency_hz,
        )
    )
    helios.set_name(HeliosConfig.SYSTEM_NAME)

    if helios_config.publish_rgb_frequency_hz > 0.0:
        rgb_publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=RGBImageData,
                channel=AegisChannel.RGB_IMAGE,
                lcm=lcm,
                publish_period=1.0 / helios_config.publish_rgb_frequency_hz,
            )
        )
        builder.Connect(
            helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE),
            rgb_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )

    if helios_config.publish_depth_frequency_hz > 0.0:
        depth_publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=DepthImageData,
                channel=AegisChannel.DEPTH_IMAGE,
                lcm=lcm,
                publish_period=1.0 / helios_config.publish_depth_frequency_hz,
            )
        )
        builder.Connect(
            helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE),
            depth_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )

    diagram = builder.Build()
    diagram.set_name("aegis_helios_process")

    backend.start()
    try:
        simulator = Simulator(diagram)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        advance_until_signal(simulator)
    finally:
        backend.stop()


def _main() -> None:
    from manor.common.aegis.aegis import AegisConfig

    raw = json.loads(sys.stdin.read())
    config = AegisConfig.from_yaml_dict(raw)
    run_helios(helios_config=config.helios_config)


if __name__ == "__main__":
    _main()
