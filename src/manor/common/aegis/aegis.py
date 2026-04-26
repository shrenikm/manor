"""
Aegis top-level assembly.

Builds the full Diagram out of Helios, Talos, Metis, Kyber, and the
LCM publisher / subscriber adapters that connect them. The graph
shape is identical in sim and hardware modes; only the backends and a
sim-only ``GaiaAdvancer`` LeafSystem differ.

Logical data flow (LCM channels):

    Helios.rgb_image          --LCM(RGB_IMAGE)----> Metis.rgb_image
    Helios.depth_image        --LCM(DEPTH_IMAGE)--> Metis.depth_image
    Talos.proprioception      --LCM(PROPRIOCEPTION)--> Metis.proprioception
                                                       Kyber.proprioception
    Metis.action              --LCM(ACTION)------> Kyber.action
    Kyber.command             --direct----------> Talos.command

Each sub-system owns its own ``*Config`` aggregator (publish frequency,
backend choice, behavioural knobs) declared alongside the sub-system
itself; ``AegisBuildConfig`` collects them plus mode + manipulator model
+ an optional environment / Gaia config for sim mode.
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
from manor.common.aegis.gaia.env_config import EnvironmentConfig
from manor.common.aegis.gaia.gaia import Gaia, GaiaConfig
from manor.common.aegis.gaia.gaia_advancer import GaiaAdvancer, GaiaAdvancerConfig
from manor.common.aegis.helios.hardware_backend import HardwareSensorBackend
from manor.common.aegis.helios.helios import Helios, HeliosConfig, HeliosPorts, SensorBackend
from manor.common.aegis.helios.sim_backend import SimSensorBackend
from manor.common.aegis.kyber.controllers import ZeroVelocityController
from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies import ZeroVelocityPolicy
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosConfig, TalosPorts
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
class AegisBuildConfig:
    """
    Top-level configuration for ``build_aegis``.

    ``manipulator_model`` is the source of truth for which robot the
    aegis stack runs. Kyber, Talos, and (in sim mode) Gaia each
    construct an independent ``MultibodyPlant`` from it.

    ``environment_config`` and ``gaia_config`` are sim-mode only; both
    are ignored in hardware mode.
    """

    mode: AegisMode
    manipulator_model: IManipulatorModel
    helios_config: HeliosConfig = attr.field(factory=HeliosConfig)
    talos_config: TalosConfig = attr.field(factory=TalosConfig)
    metis_config: MetisConfig = attr.field(factory=MetisConfig)
    kyber_config: KyberConfig = attr.field(factory=KyberConfig)
    gaia_advancer_config: GaiaAdvancerConfig = attr.field(factory=GaiaAdvancerConfig)
    environment_config: EnvironmentConfig | None = None
    gaia_config: GaiaConfig | None = None
    lcm: DrakeLcm | None = None

    @classmethod
    def from_lite6(cls, mode: AegisMode, variant: Lite6Variant) -> Self:
        """
        Convenience: build config wrapping a Lite6Model of the given
        variant. The variant is required because both the normal and
        reverse parallel-gripper trims (and the vacuum trim) are real
        deployment options; defaulting one over the other would only
        bury the choice.
        """
        return cls(
            mode=mode,
            manipulator_model=Lite6Model(variant=variant),
        )


@attr.frozen
class AegisSystems:
    """
    Handles to each sub-system in the built diagram. ``gaia`` /
    ``gaia_advancer`` are populated only in sim mode.
    """

    helios: Helios
    talos: Talos
    metis: Metis
    kyber: Kyber
    lcm: DrakeLcm
    gaia: Gaia | None = None
    gaia_advancer: GaiaAdvancer | None = None


def build_aegis(config: AegisBuildConfig) -> tuple[Diagram, AegisSystems]:
    """
    Build and wire the full aegis diagram.

    The graph shape is identical in sim and hardware modes; only the
    backends differ, and sim mode adds a ``GaiaAdvancer`` to drive the
    shared ``Gaia`` instance forward in step with the diagram clock.
    """

    lcm = config.lcm if config.lcm is not None else DrakeLcm()
    policy = config.metis_config.policy or ZeroVelocityPolicy(num_joints=config.manipulator_model.get_num_dof())
    controller = config.kyber_config.controller or ZeroVelocityController(
        num_dof=config.manipulator_model.get_num_dof()
    )

    gaia, sensor_backend, manipulator_backend = _build_backends(config)

    builder = DiagramBuilder()
    # One LcmInterfaceSystem at the top services every subscriber adapter.
    builder.AddSystem(LcmInterfaceSystem(lcm))

    helios = builder.AddSystem(
        Helios(
            backend=sensor_backend,
            publish_rgb_frequency_hz=config.helios_config.publish_rgb_frequency_hz,
            publish_depth_frequency_hz=config.helios_config.publish_depth_frequency_hz,
        )
    )
    talos = builder.AddSystem(
        Talos(
            backend=manipulator_backend,
            manipulator_model=config.manipulator_model,
            publish_frequency=config.talos_config.publish_frequency_hz,
        )
    )
    metis = builder.AddSystem(Metis(policy=policy, publish_frequency=config.metis_config.publish_frequency_hz))
    kyber = builder.AddSystem(
        Kyber(
            controller=controller,
            manipulator_model=config.manipulator_model,
            publish_frequency=config.kyber_config.publish_frequency_hz,
        )
    )

    helios.set_name(HeliosConfig.SYSTEM_NAME)
    talos.set_name(TalosConfig.SYSTEM_NAME)
    metis.set_name(MetisConfig.SYSTEM_NAME)
    kyber.set_name(KyberConfig.SYSTEM_NAME)

    # LCM publisher / subscriber adapters. RGB / depth are conditional
    # on the matching Helios stream being enabled (frequency > 0).
    proprioception_publisher = _add_publisher(
        builder, Proprioception, AegisChannel.PROPRIOCEPTION, lcm, config.talos_config.publish_frequency_hz
    )
    action_publisher = _add_publisher(
        builder, Action, AegisChannel.ACTION, lcm, config.metis_config.publish_frequency_hz
    )
    proprioception_subscriber = _add_subscriber(builder, Proprioception, AegisChannel.PROPRIOCEPTION, lcm)
    action_subscriber = _add_subscriber(builder, Action, AegisChannel.ACTION, lcm)

    if config.helios_config.publish_rgb_frequency_hz > 0.0:
        rgb_publisher = _add_publisher(
            builder, RGBImageData, AegisChannel.RGB_IMAGE, lcm, config.helios_config.publish_rgb_frequency_hz
        )
        rgb_subscriber = _add_subscriber(builder, RGBImageData, AegisChannel.RGB_IMAGE, lcm)
        builder.Connect(
            helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE),
            rgb_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )
        builder.Connect(
            rgb_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
            metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE),
        )

    if config.helios_config.publish_depth_frequency_hz > 0.0:
        depth_publisher = _add_publisher(
            builder, DepthImageData, AegisChannel.DEPTH_IMAGE, lcm, config.helios_config.publish_depth_frequency_hz
        )
        depth_subscriber = _add_subscriber(builder, DepthImageData, AegisChannel.DEPTH_IMAGE, lcm)
        builder.Connect(
            helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE),
            depth_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )
        builder.Connect(
            depth_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
            metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE),
        )

    builder.Connect(
        talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
        proprioception_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )
    builder.Connect(
        metis.GetOutputPort(MetisPorts.OUTPUT_ACTION),
        action_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
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

    gaia_advancer: GaiaAdvancer | None = None
    if gaia is not None:
        gaia_advancer = builder.AddSystem(
            GaiaAdvancer(gaia=gaia, advance_frequency_hz=config.gaia_advancer_config.advance_frequency_hz)
        )
        gaia_advancer.set_name(GaiaAdvancerConfig.SYSTEM_NAME)

    diagram = builder.Build()
    diagram.set_name(f"aegis_{config.mode.value}")

    return diagram, AegisSystems(
        helios=helios,
        talos=talos,
        metis=metis,
        kyber=kyber,
        lcm=lcm,
        gaia=gaia,
        gaia_advancer=gaia_advancer,
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
) -> tuple[Gaia | None, SensorBackend, ManipulatorBackend]:
    if config.mode == AegisMode.SIM:
        gaia = Gaia(
            manipulator_model=config.manipulator_model,
            environment_config=config.environment_config or EnvironmentConfig.default(),
            config=config.gaia_config or GaiaConfig(),
        )
        gaia.finalize()
        sensor_backend: SensorBackend = SimSensorBackend(gaia=gaia, config=config.helios_config.sim_backend_config)
        manipulator_backend: ManipulatorBackend = SimManipulatorBackend(
            gaia=gaia, config=config.talos_config.sim_backend_config
        )
        return gaia, sensor_backend, manipulator_backend

    if config.mode == AegisMode.HARDWARE:
        # Lite6 is the only manipulator currently supported on hardware;
        # additional manipulators will need their own driver factories
        # plumbed in alongside this branch.
        if not isinstance(config.manipulator_model, Lite6Model):
            raise InvalidDefinitionError(
                f"Hardware mode currently supports only Lite6Model; got {type(config.manipulator_model).__name__}"
            )
        driver = Lite6Driver(model=config.manipulator_model)
        manipulator_backend = HardwareManipulatorBackend(
            driver=driver, config=config.talos_config.hardware_backend_config
        )
        sensor_backend = HardwareSensorBackend(config=config.helios_config.hardware_backend_config)
        return None, sensor_backend, manipulator_backend

    raise InvalidDefinitionError(f"Unknown AegisMode: {config.mode!r}")
