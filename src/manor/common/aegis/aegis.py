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

from enum import StrEnum
from typing import Self

import attr
import yaml
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
from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerManager
from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies.policy_manager import MetisPolicyManager
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosConfig, TalosPorts
from manor.common.custom_types import FilePath
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.exceptions import AegisConfigError, InvalidDefinitionError
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import get_variant_class


class AegisYamlKey(StrEnum):
    """
    Top-level YAML keys for an aegis config.
    """

    MODE = "mode"
    MANIPULATOR = "manipulator"
    ENVIRONMENT_CONFIG = "environment_config"
    HELIOS = "helios"
    TALOS = "talos"
    METIS = "metis"
    KYBER = "kyber"
    GAIA = "gaia"
    GAIA_ADVANCER = "gaia_advancer"


class _ManipulatorYamlKey(StrEnum):
    TYPE = "type"
    VARIANT = "variant"


@attr.frozen
class AegisBuildConfig:
    """
    Top-level configuration for ``build_aegis``.

    ``manipulator_model`` is the source of truth for which robot the
    aegis stack runs. Talos and (in sim mode) Gaia each construct an
    independent ``MultibodyPlant`` from it; controllers that need a
    plant build their own through ``KyberControllerManager``.

    ``metis_config`` and ``kyber_config`` are required: they pin the
    policy and controller that run on the robot, and aegis refuses to
    default either of those at the top level.
    ``environment_config`` and ``gaia_config`` are sim-mode only; both
    are ignored in hardware mode.
    """

    mode: AegisMode
    manipulator_model: IManipulatorModel
    metis_config: MetisConfig
    kyber_config: KyberConfig
    environment_config: EnvironmentConfig | None = None
    helios_config: HeliosConfig = attr.field(factory=HeliosConfig)
    talos_config: TalosConfig = attr.field(factory=TalosConfig)
    gaia_advancer_config: GaiaAdvancerConfig = attr.field(factory=GaiaAdvancerConfig)
    gaia_config: GaiaConfig | None = None
    lcm: DrakeLcm | None = None

    @classmethod
    def from_lite6(
        cls,
        mode: AegisMode,
        variant: Lite6Variant,
        metis_config: MetisConfig,
        kyber_config: KyberConfig,
    ) -> Self:
        """
        Convenience: build config wrapping a Lite6Model of the given
        variant. The variant is required because both the normal and
        reverse parallel-gripper trims (and the vacuum trim) are real
        deployment options; defaulting one over the other would only
        bury the choice. ``metis_config`` and ``kyber_config`` are
        also required here for the same reason.
        """
        return cls(
            mode=mode,
            manipulator_model=Lite6Model(variant=variant),
            metis_config=metis_config,
            kyber_config=kyber_config,
        )

    @classmethod
    def from_yaml(cls, filepath: FilePath) -> Self:
        """
        Load an ``AegisBuildConfig`` from a YAML file.

        The top-level YAML mirrors the attrs field layout: ``mode``
        and ``manipulator`` are required, and each subsystem's block
        is delegated to that subsystem's own ``from_yaml_dict``. This
        keeps the parsing logic colocated with each subsystem so the
        same blocks can be reused if a single subsystem is loaded
        from an isolated YAML in the future.
        """
        try:
            with open(filepath, "r") as fp:
                raw = yaml.safe_load(fp) or {}
        except OSError as e:
            raise AegisConfigError(f"Failed to read aegis config {filepath!r}: {e}") from e
        except yaml.YAMLError as e:
            raise AegisConfigError(f"Failed to parse aegis config {filepath!r}: {e}") from e

        if not isinstance(raw, dict):
            raise AegisConfigError(
                f"Aegis config {filepath!r} must be a mapping at the top level; got {type(raw).__name__}"
            )

        return cls.from_yaml_dict(raw)

    @classmethod
    def from_yaml_dict(cls, raw: dict) -> Self:
        """
        Build an ``AegisBuildConfig`` from an already-parsed YAML mapping.
        """
        allowed = {key.value for key in AegisYamlKey}
        extras = set(raw) - allowed
        if extras:
            raise AegisConfigError(f"aegis: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}")

        mode = _parse_mode(raw)
        manipulator_model = _parse_manipulator(raw)

        metis_raw = raw.get(AegisYamlKey.METIS)
        if not isinstance(metis_raw, dict):
            raise AegisConfigError(
                f"aegis.{AegisYamlKey.METIS} is required and must be a mapping; got {type(metis_raw).__name__}"
            )
        metis_config = MetisConfig.from_yaml_dict(metis_raw)

        kyber_raw = raw.get(AegisYamlKey.KYBER)
        if not isinstance(kyber_raw, dict):
            raise AegisConfigError(
                f"aegis.{AegisYamlKey.KYBER} is required and must be a mapping; got {type(kyber_raw).__name__}"
            )
        kyber_config = KyberConfig.from_yaml_dict(kyber_raw)

        environment_config = _parse_optional_dict_block(
            raw, AegisYamlKey.ENVIRONMENT_CONFIG, EnvironmentConfig.from_yaml_dict
        )
        helios_config = _parse_optional_dict_block(raw, AegisYamlKey.HELIOS, HeliosConfig.from_yaml_dict)
        talos_config = _parse_optional_dict_block(raw, AegisYamlKey.TALOS, TalosConfig.from_yaml_dict)
        gaia_advancer_config = _parse_optional_dict_block(
            raw, AegisYamlKey.GAIA_ADVANCER, GaiaAdvancerConfig.from_yaml_dict
        )
        gaia_config = _parse_optional_dict_block(raw, AegisYamlKey.GAIA, GaiaConfig.from_yaml_dict)

        return cls(
            mode=mode,
            manipulator_model=manipulator_model,
            metis_config=metis_config,
            kyber_config=kyber_config,
            environment_config=environment_config,
            helios_config=helios_config if helios_config is not None else HeliosConfig(),
            talos_config=talos_config if talos_config is not None else TalosConfig(),
            gaia_advancer_config=gaia_advancer_config if gaia_advancer_config is not None else GaiaAdvancerConfig(),
            gaia_config=gaia_config,
        )


def _parse_mode(raw: dict) -> AegisMode:
    mode_value = raw.get(AegisYamlKey.MODE)
    if not isinstance(mode_value, str) or not mode_value:
        raise AegisConfigError(f"aegis.{AegisYamlKey.MODE} is required and must be a non-empty string")
    try:
        return AegisMode(mode_value)
    except ValueError as e:
        raise AegisConfigError(
            f"Unknown aegis.{AegisYamlKey.MODE}: {mode_value!r}; expected one of {[m.value for m in AegisMode]}"
        ) from e


def _parse_manipulator(raw: dict) -> IManipulatorModel:
    manipulator_raw = raw.get(AegisYamlKey.MANIPULATOR)
    if not isinstance(manipulator_raw, dict):
        raise AegisConfigError(
            f"aegis.{AegisYamlKey.MANIPULATOR} is required and must be a mapping; got {type(manipulator_raw).__name__}"
        )

    allowed = {key.value for key in _ManipulatorYamlKey}
    extras = set(manipulator_raw) - allowed
    if extras:
        raise AegisConfigError(
            f"aegis.{AegisYamlKey.MANIPULATOR}: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}"
        )

    type_value = manipulator_raw.get(_ManipulatorYamlKey.TYPE)
    if not isinstance(type_value, str) or not type_value:
        raise AegisConfigError(
            f"aegis.{AegisYamlKey.MANIPULATOR}.{_ManipulatorYamlKey.TYPE} is required and must be a non-empty string"
        )
    try:
        manipulator_type = ManipulatorType(type_value)
    except ValueError as e:
        raise AegisConfigError(
            f"Unknown manipulator type {type_value!r}; expected one of {[t.value for t in ManipulatorType]}"
        ) from e

    variant_value = manipulator_raw.get(_ManipulatorYamlKey.VARIANT)
    if not isinstance(variant_value, str) or not variant_value:
        raise AegisConfigError(
            f"aegis.{AegisYamlKey.MANIPULATOR}.{_ManipulatorYamlKey.VARIANT} is required and must be a non-empty string"
        )

    variant_cls = get_variant_class(manipulator_type)
    try:
        variant = variant_cls(variant_value)
    except ValueError as e:
        raise AegisConfigError(
            f"Unknown variant {variant_value!r} for manipulator {manipulator_type!r}; "
            f"expected one of {[v.value for v in variant_cls]}"
        ) from e

    if manipulator_type is ManipulatorType.LITE6:
        assert isinstance(variant, Lite6Variant)
        return Lite6Model(variant=variant)
    raise AegisConfigError(f"No model factory wired in for manipulator {manipulator_type!r}")


def _parse_optional_dict_block(raw: dict, key: AegisYamlKey, parser):
    """
    Run ``parser`` on the dict at ``raw[key]`` if present; return
    ``None`` when absent. Errors out cleanly if the value isn't a
    mapping. ``parser`` is expected to be a ``from_yaml_dict``-style
    classmethod.
    """
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, dict):
        raise AegisConfigError(f"aegis.{key} must be a mapping; got {type(value).__name__}")
    return parser(value)


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
    policy = MetisPolicyManager.from_config(config.metis_config.policy_config)
    controller = KyberControllerManager.from_config(
        config.kyber_config.controller_config,
        manipulator_model=config.manipulator_model,
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
