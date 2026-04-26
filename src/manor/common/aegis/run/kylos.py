"""
Kylos-process runner (hardware mode only).

Bundles ``Kyber`` + ``Talos`` (with the hardware manipulator backend)
in one process. Action arrives over LCM; proprioception is published
over LCM. Kyber and Talos are wired directly to each other -- no
intermediate LCM hop on the action -> command -> motor path.

In sim mode the kylos role is fulfilled by the gylos process (see
``run.gylos``); don't launch this runner in sim.

Standalone usage:

    python -m manor.common.aegis.run.kylos < lite6_default.yaml.json
"""

from __future__ import annotations

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
from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerManager
from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
from manor.common.aegis.run.run_utils import advance_until_signal
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend
from manor.common.aegis.talos.talos import Talos, TalosConfig, TalosPorts
from manor.common.definitions.action import Action
from manor.common.definitions.proprioception import Proprioception
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.manipulator_model import IManipulatorModel


def run_kylos(
    manipulator_model: IManipulatorModel,
    talos_config: TalosConfig,
    kyber_config: KyberConfig,
    lcm: DrakeLcm | None = None,
) -> None:
    """
    Build the kylos-process diagram (kyber + talos against the real
    robot driver) and advance forever (until SIGTERM / SIGINT).
    """
    lcm = lcm if lcm is not None else DrakeLcm()

    # Lite6 is the only manipulator currently driveable on hardware;
    # additional manipulators will need their own driver factories
    # plumbed in alongside this branch.
    if not isinstance(manipulator_model, Lite6Model):
        raise ValueError(
            f"kylos hardware mode currently supports only Lite6Model; got {type(manipulator_model).__name__}"
        )
    driver = Lite6Driver(model=manipulator_model)
    backend = HardwareManipulatorBackend(driver=driver, config=talos_config.hardware_backend_config)
    controller = KyberControllerManager.from_config(kyber_config.controller_config, manipulator_model=manipulator_model)

    builder = DiagramBuilder()
    builder.AddSystem(LcmInterfaceSystem(lcm))

    talos = builder.AddSystem(
        Talos(
            backend=backend,
            manipulator_model=manipulator_model,
            publish_frequency=talos_config.publish_frequency_hz,
        )
    )
    talos.set_name(TalosConfig.SYSTEM_NAME)

    kyber = builder.AddSystem(Kyber(controller=controller, publish_frequency=kyber_config.publish_frequency_hz))
    kyber.set_name(KyberConfig.SYSTEM_NAME)

    action_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(definition_cls=Action, channel=AegisChannel.ACTION, lcm=lcm)
    )
    builder.Connect(
        action_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        kyber.GetInputPort(KyberPorts.INPUT_ACTION),
    )

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

    builder.Connect(
        kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND),
        talos.GetInputPort(TalosPorts.INPUT_COMMAND),
    )

    diagram = builder.Build()
    diagram.set_name("aegis_kylos_process")

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
    run_kylos(
        manipulator_model=config.manipulator_model,
        talos_config=config.talos_config,
        kyber_config=config.kyber_config,
    )


if __name__ == "__main__":
    _main()
