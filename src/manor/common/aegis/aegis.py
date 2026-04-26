"""
Aegis top-level assembly.

Builds the full Diagram out of Helios, Talos, Metis, Kyber, and the
LCM publisher / subscriber adapters that connect them. The graph
shape is identical in sim and hardware modes; only the backends and a
sim-only ``SimAdvancer`` LeafSystem differ.

Logical data flow (LCM channels):

    Helios.rgb_image          --LCM(RGB_IMAGE)----> Metis.rgb_image
    Helios.depth_image        --LCM(DEPTH_IMAGE)--> Metis.depth_image
    Talos.proprioception      --LCM(PROPRIOCEPTION)--> Metis.proprioception
                                                       Kyber.proprioception
    Metis.action              --LCM(ACTION)------> Kyber.action
    Kyber.command             --direct----------> Talos.command

Helios, Talos, Metis are direct-publishers + subscribers via LCM;
Kyber's command output is wired straight into Talos (no LCM hop).
"""

from __future__ import annotations

from typing import Self

import attr
from pydrake.lcm import DrakeLcm
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import (
    AegisAdapterPorts,
    AegisLCMPublisherAdapter,
    AegisLCMSubscriberAdapter,
)
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.aegis.helios.hardware_backend import HardwareSensorBackend, HardwareSensorBackendConfig
from manor.common.aegis.helios.helios import Helios, HeliosPorts, SensorBackend
from manor.common.aegis.helios.sim_backend import SimSensorBackend, SimSensorBackendConfig
from manor.common.aegis.kyber.kyber import Kyber, KyberPorts
from manor.common.aegis.metis.metis import Metis, MetisPorts, Policy
from manor.common.aegis.metis.policies import ZeroVelocityPolicy
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.sim.env_config import EnvironmentConfig
from manor.common.aegis.sim.sim import Sim, SimConfig
from manor.common.aegis.sim.sim_advancer import SimAdvancer
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend, HardwareManipulatorBackendConfig
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend, SimManipulatorBackendConfig
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosPorts
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.exceptions import InvalidDefinitionError
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_model import IManipulatorModel


@attr.frozen
class AegisFrequencies:
    """
    Per-subsystem publish rates in Hz.

    Defaults are order-of-magnitude sensible for a research manipulation
    stack (cameras ~30 Hz, the actuation + state loop in the hundreds, a
    high-level policy in the tens, low-level control the fastest).
    """

    helios_hz: float = 30.0
    talos_hz: float = 200.0
    metis_hz: float = 10.0
    kyber_hz: float = 500.0
    sim_advancer_hz: float = 500.0


@attr.frozen
class AegisSystems:
    """
    Handles to each sub-system in the built diagram. Useful for tests
    and for wiring external driver / recorder systems after
    ``build_aegis`` returns. ``sim`` is populated only in sim mode.
    """

    helios: Helios
    talos: Talos
    metis: Metis
    kyber: Kyber
    lcm: DrakeLcm
    sim: Sim | None = None
    sim_advancer: SimAdvancer | None = None


@attr.frozen
class AegisBuildConfig:
    """
    Configuration for ``build_aegis``.

    ``manipulator_model`` is the source of truth for which robot the
    aegis stack runs. Kyber, Talos, and (in sim mode) Sim each
    construct an independent ``MultibodyPlant`` from it. ``policy``
    defaults to a no-op zero-velocity policy.

    ``environment_config`` and ``sim_config`` are sim-mode only; both
    are ignored in hardware mode.
    """

    mode: AegisMode
    manipulator_model: IManipulatorModel
    policy: Policy | None = None
    environment_config: EnvironmentConfig | None = None
    sim_config: SimConfig | None = None
    frequencies: AegisFrequencies = attr.field(factory=AegisFrequencies)
    lcm: DrakeLcm | None = None

    @classmethod
    def default_lite6(cls, mode: AegisMode) -> Self:
        """
        Convenience: build config with a default Lite6 + zero-velocity
        policy.
        """
        return cls(
            mode=mode,
            manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL),
        )


def build_aegis(config: AegisBuildConfig) -> tuple[Diagram, AegisSystems]:
    """
    Build and wire the full aegis diagram.

    The graph shape is identical in sim and hardware modes; only the
    backends differ, and sim mode adds a ``SimAdvancer`` to drive the
    shared ``Sim`` instance forward in step with the diagram clock.
    """

    lcm = config.lcm if config.lcm is not None else DrakeLcm()
    policy = (
        config.policy
        if config.policy is not None
        else ZeroVelocityPolicy(num_joints=config.manipulator_model.get_num_dof())
    )

    sim, sensor_backend, manipulator_backend = _build_backends(config)

    builder = DiagramBuilder()
    # One LcmInterfaceSystem at the top services every subscriber adapter.
    builder.AddSystem(LcmInterfaceSystem(lcm))

    helios = builder.AddSystem(Helios(backend=sensor_backend, publish_frequency=config.frequencies.helios_hz))
    talos = builder.AddSystem(
        Talos(
            backend=manipulator_backend,
            manipulator_model=config.manipulator_model,
            publish_frequency=config.frequencies.talos_hz,
        )
    )
    metis = builder.AddSystem(Metis(policy=policy, publish_frequency=config.frequencies.metis_hz))
    kyber = builder.AddSystem(
        Kyber(
            manipulator_model=config.manipulator_model,
            publish_frequency=config.frequencies.kyber_hz,
        )
    )

    helios.set_name("helios")
    talos.set_name("talos")
    metis.set_name("metis")
    kyber.set_name("kyber")

    # LCM publisher / subscriber adapters. Helios and Talos publish;
    # Metis subscribes to images + proprioception and publishes its
    # action; Kyber subscribes to actions and to proprioception (the
    # latter is also direct-needed; we route via LCM here for parity).
    rgb_publisher = _add_publisher(builder, RGBImageData, AegisChannel.RGB_IMAGE, lcm, config.frequencies.helios_hz)
    depth_publisher = _add_publisher(
        builder, DepthImageData, AegisChannel.DEPTH_IMAGE, lcm, config.frequencies.helios_hz
    )
    proprioception_publisher = _add_publisher(
        builder, Proprioception, AegisChannel.PROPRIOCEPTION, lcm, config.frequencies.talos_hz
    )
    action_publisher = _add_publisher(builder, Action, AegisChannel.ACTION, lcm, config.frequencies.metis_hz)

    rgb_subscriber = _add_subscriber(builder, RGBImageData, AegisChannel.RGB_IMAGE, lcm)
    depth_subscriber = _add_subscriber(builder, DepthImageData, AegisChannel.DEPTH_IMAGE, lcm)
    proprioception_subscriber = _add_subscriber(builder, Proprioception, AegisChannel.PROPRIOCEPTION, lcm)
    action_subscriber = _add_subscriber(builder, Action, AegisChannel.ACTION, lcm)

    # Producers -> publishers.
    builder.Connect(
        helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE),
        rgb_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )
    builder.Connect(
        helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE),
        depth_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )
    builder.Connect(
        talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
        proprioception_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )
    builder.Connect(
        metis.GetOutputPort(MetisPorts.OUTPUT_ACTION),
        action_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )

    # Subscribers -> consumers.
    builder.Connect(
        rgb_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE),
    )
    builder.Connect(
        depth_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE),
    )
    builder.Connect(
        proprioception_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        metis.GetInputPort(MetisPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        proprioception_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        action_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        kyber.GetInputPort(KyberPorts.INPUT_ACTION),
    )

    # Direct: Kyber.command -> Talos.command (no LCM hop, as planned).
    builder.Connect(
        kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND),
        talos.GetInputPort(TalosPorts.INPUT_COMMAND),
    )

    sim_advancer: SimAdvancer | None = None
    if sim is not None:
        sim_advancer = builder.AddSystem(SimAdvancer(sim=sim, advance_frequency_hz=config.frequencies.sim_advancer_hz))
        sim_advancer.set_name("sim_advancer")

    diagram = builder.Build()
    diagram.set_name(f"aegis_{config.mode.value}")

    return diagram, AegisSystems(
        helios=helios,
        talos=talos,
        metis=metis,
        kyber=kyber,
        lcm=lcm,
        sim=sim,
        sim_advancer=sim_advancer,
    )


def _add_publisher(
    builder: DiagramBuilder,
    definition_cls: type,
    channel: AegisChannel,
    lcm: DrakeLcm,
    publish_frequency_hz: float,
) -> AegisLCMPublisherAdapter:
    return builder.AddSystem(
        AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=definition_cls,
            channel=channel,
            lcm=lcm,
            publish_period=1.0 / publish_frequency_hz,
        )
    )


def _add_subscriber(
    builder: DiagramBuilder,
    definition_cls: type,
    channel: AegisChannel,
    lcm: DrakeLcm,
) -> AegisLCMSubscriberAdapter:
    return builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(
            definition_cls=definition_cls,
            channel=channel,
            lcm=lcm,
        )
    )


def _build_backends(
    config: AegisBuildConfig,
) -> tuple[Sim | None, SensorBackend, ManipulatorBackend]:
    if config.mode == AegisMode.SIM:
        sim = Sim(
            manipulator_model=config.manipulator_model,
            environment_config=config.environment_config,
            config=config.sim_config,
        )
        sensor_backend: SensorBackend = SimSensorBackend(sim=sim, config=SimSensorBackendConfig())
        manipulator_backend: ManipulatorBackend = SimManipulatorBackend(sim=sim, config=SimManipulatorBackendConfig())
        return sim, sensor_backend, manipulator_backend

    if config.mode == AegisMode.HARDWARE:
        # Lite6 is the only manipulator currently supported on hardware;
        # additional manipulators will need their own driver factories
        # plumbed in alongside this branch.
        if not isinstance(config.manipulator_model, Lite6Model):
            raise InvalidDefinitionError(
                f"Hardware mode currently supports only Lite6Model; got {type(config.manipulator_model).__name__}"
            )
        driver = Lite6Driver(model=config.manipulator_model)
        manipulator_backend = HardwareManipulatorBackend(driver=driver, config=HardwareManipulatorBackendConfig())
        sensor_backend = HardwareSensorBackend(config=HardwareSensorBackendConfig())
        return None, sensor_backend, manipulator_backend

    raise InvalidDefinitionError(f"Unknown AegisMode: {config.mode!r}")
