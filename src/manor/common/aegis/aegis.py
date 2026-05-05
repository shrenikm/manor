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
itself; ``AegisConfig`` collects them plus mode + manipulator model
+ an optional environment / Gaia config for sim mode. The canonical
construction path is ``AegisConfig.from_yaml`` -- programmatic
construction works too, but the YAML is the source of truth for the
config schema.
"""

from __future__ import annotations

from pathlib import Path
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
from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerManager, KyberControllerType
from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies.policy_manager import MetisPolicyManager, MetisPolicyType
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosConfig, TalosPorts
from manor.common.aegis.yaml_utils import parse_attrs_yaml, require_str
from manor.common.custom_types import FilePath
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.exceptions import AegisConfigError, InvalidDefinitionError
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import get_variant_class

# Discriminated reference to a specific manipulator + variant; lives
# inside the ``manipulator_model:`` YAML block. Kept inline because
# this is the only block whose YAML keys don't correspond to attrs
# fields on a config class -- the keys ``type`` / ``variant`` map
# directly onto ``ManipulatorType`` and the registered variant enum.
_MANIPULATOR_TYPE_KEY = "type"
_MANIPULATOR_VARIANT_KEY = "variant"
_MANIPULATOR_ALLOWED_KEYS = {_MANIPULATOR_TYPE_KEY, _MANIPULATOR_VARIANT_KEY}

# Layout for the split base / policy / controller YAML scheme. The
# base YAML (e.g. ``lite6_ac.yaml``) lives directly under
# ``configs/aegis/`` and references a policy and controller by name;
# the per-policy and per-controller YAMLs live in sibling
# subdirectories so the type-name -> filename mapping is mechanical.
AEGIS_YAML_SUFFIX = "_ac.yaml"
POLICIES_SUBDIR = "policies"
CONTROLLERS_SUBDIR = "controllers"

# YAML keys used by the composer. The base YAML carries
# ``policy_type`` / ``controller_type`` strings; after composition
# these are replaced with fully-inlined ``policy_config`` /
# ``controller_config`` blocks (with a synthesised ``type:`` tag) so
# that the existing ``MetisConfig`` / ``KyberConfig`` parsing path,
# which dispatches off ``type:``, works unchanged.
_POLICY_TYPE_KEY = "policy_type"
_CONTROLLER_TYPE_KEY = "controller_type"
_POLICY_CONFIG_KEY = "policy_config"
_CONTROLLER_CONFIG_KEY = "controller_config"
_TYPE_DISCRIMINATOR_KEY = "type"
_METIS_CONFIG_KEY = "metis_config"
_KYBER_CONFIG_KEY = "kyber_config"


def _read_yaml_mapping(path: Path, context: str) -> dict:
    """
    Read a YAML file from disk and validate that its top-level value
    is a mapping. Empty files round-trip to an empty mapping rather
    than ``None`` so callers can rely on a uniform dict shape.
    """
    try:
        with open(path, "r") as fp:
            raw = yaml.safe_load(fp)
    except OSError as e:
        raise AegisConfigError(f"Failed to read {context} {str(path)!r}: {e}") from e
    except yaml.YAMLError as e:
        raise AegisConfigError(f"Failed to parse {context} {str(path)!r}: {e}") from e
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise AegisConfigError(f"{context} {str(path)!r} must be a mapping at the top level; got {type(raw).__name__}")
    return raw


def _resolve_typed_yaml_block(
    block: dict,
    block_name: str,
    type_key: str,
    config_key: str,
    subdir: Path,
    valid_type_values: set[str],
) -> None:
    """
    Translate a base-YAML block of the form ``{<type_key>: <name>, ...}``
    into the inlined ``{<config_key>: {type: <name>, ...sub_body}, ...}``
    shape the existing aegis parsers expect. Mutates ``block`` in
    place.

    The plan: every metis_config / kyber_config block in the base
    YAML names a policy / controller via ``policy_type`` /
    ``controller_type``; the matching ``<name>_ac.yaml`` body lives
    under ``configs/aegis/policies/`` or ``configs/aegis/controllers/``;
    aegis refuses to run if either file is missing. The composer is
    the single place those rules are enforced.
    """
    if config_key in block:
        raise AegisConfigError(
            f"{block_name}.{config_key} must not be set in the base YAML; "
            f"set {block_name}.{type_key} instead and put the body in "
            f"{subdir.name}/<{type_key}>{AEGIS_YAML_SUFFIX}"
        )
    if type_key not in block:
        raise AegisConfigError(f"{block_name}.{type_key} is required and must name a YAML in {subdir.name}/")
    type_value = block.pop(type_key)
    if not isinstance(type_value, str) or not type_value:
        raise AegisConfigError(f"{block_name}.{type_key} must be a non-empty string; got {type_value!r}")
    if type_value not in valid_type_values:
        raise AegisConfigError(
            f"Unknown {block_name}.{type_key}: {type_value!r}; expected one of {sorted(valid_type_values)}"
        )

    sub_path = subdir / f"{type_value}{AEGIS_YAML_SUFFIX}"
    if not sub_path.exists():
        raise AegisConfigError(
            f"{block_name}.{type_key}={type_value!r} but {sub_path} is missing; "
            f"every policy / controller must have a YAML defined for it"
        )

    sub_body = _read_yaml_mapping(sub_path, context=f"{block_name}.{type_key}={type_value!r} body")
    if _TYPE_DISCRIMINATOR_KEY in sub_body:
        raise AegisConfigError(
            f"{sub_path}: must not contain a {_TYPE_DISCRIMINATOR_KEY!r} key; the type is derived from the filename"
        )

    block[config_key] = {_TYPE_DISCRIMINATOR_KEY: type_value, **sub_body}


def compose_aegis_yaml_dict(base_yaml_path: FilePath) -> dict:
    """
    Read a base aegis YAML and inline its policy and controller
    sub-YAMLs into a single dict ready for ``AegisConfig.from_yaml_dict``.

    The base YAML's ``metis_config.policy_type`` and
    ``kyber_config.controller_type`` are looked up in
    ``<base.parent>/policies/`` and ``<base.parent>/controllers/``
    respectively. If a sub-YAML is missing the composer raises so
    aegis refuses to run with a config that relies on undefined
    behaviour.
    """
    base_path = Path(base_yaml_path)
    raw = _read_yaml_mapping(base_path, context="aegis base config")

    metis_block = raw.get(_METIS_CONFIG_KEY)
    if not isinstance(metis_block, dict):
        raise AegisConfigError(
            f"{_METIS_CONFIG_KEY} block is required and must be a mapping; "
            f"got {type(metis_block).__name__ if metis_block is not None else 'absent'}"
        )
    kyber_block = raw.get(_KYBER_CONFIG_KEY)
    if not isinstance(kyber_block, dict):
        raise AegisConfigError(
            f"{_KYBER_CONFIG_KEY} block is required and must be a mapping; "
            f"got {type(kyber_block).__name__ if kyber_block is not None else 'absent'}"
        )

    config_dir = base_path.parent
    _resolve_typed_yaml_block(
        block=metis_block,
        block_name=_METIS_CONFIG_KEY,
        type_key=_POLICY_TYPE_KEY,
        config_key=_POLICY_CONFIG_KEY,
        subdir=config_dir / POLICIES_SUBDIR,
        valid_type_values={t.value for t in MetisPolicyType},
    )
    _resolve_typed_yaml_block(
        block=kyber_block,
        block_name=_KYBER_CONFIG_KEY,
        type_key=_CONTROLLER_TYPE_KEY,
        config_key=_CONTROLLER_CONFIG_KEY,
        subdir=config_dir / CONTROLLERS_SUBDIR,
        valid_type_values={t.value for t in KyberControllerType},
    )
    return raw


def _parse_manipulator_model(value: object, context: str) -> IManipulatorModel:
    if not isinstance(value, dict):
        raise AegisConfigError(f"'{context}' must be a mapping; got {type(value).__name__}")
    extras = set(value) - _MANIPULATOR_ALLOWED_KEYS
    if extras:
        raise AegisConfigError(
            f"{context}: unexpected keys {sorted(extras)!r}; allowed {sorted(_MANIPULATOR_ALLOWED_KEYS)!r}"
        )

    type_value = require_str(value.get(_MANIPULATOR_TYPE_KEY), f"{context}.{_MANIPULATOR_TYPE_KEY}")
    try:
        manipulator_type = ManipulatorType(type_value)
    except ValueError as e:
        raise AegisConfigError(
            f"Unknown manipulator type {type_value!r}; expected one of {[t.value for t in ManipulatorType]}"
        ) from e

    variant_value = require_str(value.get(_MANIPULATOR_VARIANT_KEY), f"{context}.{_MANIPULATOR_VARIANT_KEY}")
    variant_cls = get_variant_class(manipulator_type)
    try:
        variant = variant_cls(variant_value)
    except ValueError as e:
        raise AegisConfigError(
            f"Unknown variant {variant_value!r} for manipulator {manipulator_type!r}; "
            f"expected one of {[v.value for v in variant_cls]}"
        ) from e

    if manipulator_type is ManipulatorType.LITE6:
        return Lite6Model(variant=variant)
    raise AegisConfigError(f"No model factory wired in for manipulator {manipulator_type!r}")


@attr.frozen
class AegisConfig:
    """
    Top-level configuration for ``build_aegis``.

    ``manipulator_model`` is the source of truth for which robot the
    aegis stack runs. Talos and (in sim mode) Gaia each construct an
    independent ``MultibodyPlant`` from it; controllers that need a
    plant build their own through ``KyberControllerManager``.

    Every sub-config is required -- the YAML is the explicit source
    of truth for the full schema. Sim-only blocks
    (``environment_config``, ``gaia_config``, ``gaia_advancer_config``)
    must still be present in hardware-mode YAMLs (their fields all
    default cleanly, so an empty mapping ``{}`` is valid); aegis just
    ignores them when building the hardware diagram.

    The canonical construction path is ``AegisConfig.from_yaml``; the
    raw ``__init__`` exists for programmatic use (notably tests) but
    is not the documented entry point for end users.
    """

    mode: AegisMode
    manipulator_model: IManipulatorModel
    metis_config: MetisConfig
    kyber_config: KyberConfig
    environment_config: EnvironmentConfig
    helios_config: HeliosConfig
    talos_config: TalosConfig
    gaia_advancer_config: GaiaAdvancerConfig
    gaia_config: GaiaConfig
    lcm: DrakeLcm | None = None

    @classmethod
    def from_yaml(cls, filepath: FilePath) -> Self:
        """
        Load an ``AegisConfig`` from a base YAML file. The base YAML
        names a policy and controller via ``policy_type`` and
        ``controller_type``; the matching bodies are read from
        ``<base.parent>/policies/`` and ``<base.parent>/controllers/``
        and inlined before parsing. See ``compose_aegis_yaml_dict``
        for the rules.
        """
        return cls.from_yaml_dict(compose_aegis_yaml_dict(filepath))

    @classmethod
    def from_yaml_dict(cls, raw: dict) -> Self:
        """
        Build an ``AegisConfig`` from an already-parsed YAML mapping.
        Each subsystem block is delegated to that subsystem's
        ``from_yaml_dict`` -- which the helper finds automatically by
        looking at the field type, except for ``manipulator_model``
        (a Protocol with no YAML schema of its own; needs a custom
        parser) and ``lcm`` (a runtime-only Drake handle, never set
        from YAML).
        """
        return cls(
            **parse_attrs_yaml(
                cls,
                raw,
                "aegis",
                custom_parsers={"manipulator_model": _parse_manipulator_model},
                skip_fields={"lcm"},
            )
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


def build_aegis(config: AegisConfig) -> tuple[Diagram, AegisSystems]:
    """
    Build and wire the full aegis diagram.

    The graph shape is identical in sim and hardware modes; only the
    backends differ, and sim mode adds a ``GaiaAdvancer`` to drive the
    shared ``Gaia`` instance forward in step with the diagram clock.
    """

    lcm = config.lcm if config.lcm is not None else DrakeLcm()
    policy = MetisPolicyManager.from_config(
        config.metis_config.policy_config,
        manipulator_model=config.manipulator_model,
    )
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
        kyber.GetOutputPort(KyberPorts.OUTPUT_JOINT_EE_COMMAND),
        talos.GetInputPort(TalosPorts.INPUT_JOINT_EE_COMMAND),
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
    config: AegisConfig,
) -> tuple[Gaia | None, SensorBackend, ManipulatorBackend]:
    if config.mode == AegisMode.SIM:
        gaia = Gaia(
            manipulator_model=config.manipulator_model,
            environment_config=config.environment_config,
            config=config.gaia_config,
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
        driver = Lite6Driver(
            model=config.manipulator_model,
            config=config.talos_config.hardware_backend_config.lite6_driver_config,
        )
        manipulator_backend = HardwareManipulatorBackend(
            driver=driver, config=config.talos_config.hardware_backend_config
        )
        sensor_backend = HardwareSensorBackend(config=config.helios_config.hardware_backend_config)
        return None, sensor_backend, manipulator_backend

    raise InvalidDefinitionError(f"Unknown AegisMode: {config.mode!r}")
