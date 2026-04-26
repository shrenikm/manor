"""
Gylos-process runner (sim mode).

Bundles every aegis system that needs the simulated world into a
single process: ``Gaia`` + ``GaiaAdvancer`` + ``Helios`` (sim
backend) + ``Talos`` (sim backend) + ``Kyber``.

Wiring (kyber/talos run as a tight loop with no LCM hop in between):

    LCM(ACTION)         -> Kyber.action_in
    Talos.proprioception -> Kyber.proprioception_in (direct)
    Kyber.command       -> Talos.command_in (direct)
    Talos.proprioception -> LCM(PROPRIOCEPTION)
    Helios.rgb_image    -> LCM(RGB_IMAGE)         (if rgb stream enabled)
    Helios.depth_image  -> LCM(DEPTH_IMAGE)       (if depth stream enabled)

Standalone usage:

    python -m manor.common.aegis.run.run_gylos < lite6_default.yaml.json

The JSON payload is the full parsed-AegisConfig dict (the same shape
the YAML produces). gylos just ignores ``metis_config``; everything
else feeds the diagram. Passing the full dict lets the supervisor
pre-validate the entire config once and avoids per-block payload
schemas.
"""

from __future__ import annotations

import gc
import json
import sys

from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import (
    AegisAdapterPorts,
    AegisLCMPublisherAdapter,
    AegisLCMSubscriberAdapter,
)
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.aegis.gaia.env_config import EnvironmentConfig
from manor.common.aegis.gaia.gaia import Gaia, GaiaConfig
from manor.common.aegis.gaia.gaia_advancer import GaiaAdvancer, GaiaAdvancerConfig
from manor.common.aegis.helios.helios import Helios, HeliosConfig, HeliosPorts
from manor.common.aegis.helios.sim_backend import SimSensorBackend
from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerManager
from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
from manor.common.aegis.run.run_utils import advance_until_signal
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend
from manor.common.aegis.talos.talos import Talos, TalosConfig, TalosPorts
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.manipulators.manipulator_model import IManipulatorModel


def run_gylos(
    manipulator_model: IManipulatorModel,
    environment_config: EnvironmentConfig,
    gaia_config: GaiaConfig,
    gaia_advancer_config: GaiaAdvancerConfig,
    helios_config: HeliosConfig,
    talos_config: TalosConfig,
    kyber_config: KyberConfig,
    lcm: DrakeLcm | None = None,
) -> None:
    """
    Build the gylos-process diagram, ``Initialize`` its Simulator,
    and advance forever (until SIGTERM / SIGINT). All sub-configs are
    required; the supervisor enforces presence at the YAML layer.
    """
    lcm = lcm if lcm is not None else DrakeLcm()

    gaia = Gaia(
        manipulator_model=manipulator_model,
        environment_config=environment_config,
        config=gaia_config,
    )
    gaia.finalize()

    sensor_backend = SimSensorBackend(gaia=gaia, config=helios_config.sim_backend_config)
    manipulator_backend = SimManipulatorBackend(gaia=gaia, config=talos_config.sim_backend_config)
    controller = KyberControllerManager.from_config(kyber_config.controller_config, manipulator_model=manipulator_model)

    builder = DiagramBuilder()
    builder.AddSystem(LcmInterfaceSystem(lcm))

    helios = builder.AddSystem(
        Helios(
            backend=sensor_backend,
            publish_rgb_frequency_hz=helios_config.publish_rgb_frequency_hz,
            publish_depth_frequency_hz=helios_config.publish_depth_frequency_hz,
        )
    )
    helios.set_name(HeliosConfig.SYSTEM_NAME)

    talos = builder.AddSystem(
        Talos(
            backend=manipulator_backend,
            manipulator_model=manipulator_model,
            publish_frequency=talos_config.publish_frequency_hz,
        )
    )
    talos.set_name(TalosConfig.SYSTEM_NAME)

    kyber = builder.AddSystem(Kyber(controller=controller, publish_frequency=kyber_config.publish_frequency_hz))
    kyber.set_name(KyberConfig.SYSTEM_NAME)

    gaia_advancer = builder.AddSystem(
        GaiaAdvancer(gaia=gaia, advance_frequency_hz=gaia_advancer_config.advance_frequency_hz)
    )
    gaia_advancer.set_name(GaiaAdvancerConfig.SYSTEM_NAME)

    # Action arrives from the metis process over LCM.
    action_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(definition_cls=Action, channel=AegisChannel.ACTION, lcm=lcm)
    )
    builder.Connect(
        action_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        kyber.GetInputPort(KyberPorts.INPUT_ACTION),
    )

    # Talos's proprioception drives Kyber directly (no LCM hop) and
    # is also published on LCM so the metis process can subscribe.
    builder.Connect(
        talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
        kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION),
    )
    proprioception_publisher = builder.AddSystem(
        AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=Proprioception,
            channel=AegisChannel.PROPRIOCEPTION,
            lcm=lcm,
            publish_period=1.0 / talos_config.publish_frequency_hz,
        )
    )
    builder.Connect(
        talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
        proprioception_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )

    # Kyber.command -> Talos.command, also direct.
    builder.Connect(
        kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND),
        talos.GetInputPort(TalosPorts.INPUT_COMMAND),
    )

    # Helios image streams are conditional on per-stream frequency > 0.
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
    diagram.set_name("aegis_gylos_process")

    simulator = Simulator(diagram)
    # Outer aegis simulator is the one and only place where wall-clock
    # pacing is enforced; Gaia's inner simulator runs as fast as
    # possible (see GaiaConfig.target_realtime_rate docstring).
    simulator.set_target_realtime_rate(gaia_config.target_realtime_rate)
    simulator.Initialize()
    try:
        advance_until_signal(simulator)
    finally:
        # Force the Drake Meshcat C++ destructor to run synchronously
        # here -- closing the listening socket -- so the next gylos
        # launch can re-bind port 7000. Without this, Python only
        # tears down meshcat during interpreter shutdown, by which
        # point our REPL kill-wait may have given up and started a
        # fresh gylos that finds port 7000 still held.
        gaia.shutdown()
        gc.collect()


def _main() -> None:
    # Lazy import to avoid pulling AegisConfig (and its full dep tree)
    # into every importer of this module.
    from manor.common.aegis.aegis import AegisConfig

    raw = json.loads(sys.stdin.read())
    config = AegisConfig.from_yaml_dict(raw)
    run_gylos(
        manipulator_model=config.manipulator_model,
        environment_config=config.environment_config,
        gaia_config=config.gaia_config,
        gaia_advancer_config=config.gaia_advancer_config,
        helios_config=config.helios_config,
        talos_config=config.talos_config,
        kyber_config=config.kyber_config,
    )


if __name__ == "__main__":
    _main()
