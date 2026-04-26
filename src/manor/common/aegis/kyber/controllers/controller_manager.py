"""
Kyber controller registry and factory.

Mirrors the structure of ``metis/policies/policy_manager.py``:

* ``KyberController`` -- the protocol every controller implementation
  conforms to.
* ``KyberControllerType`` -- the canonical enum used to refer to a
  controller by name (e.g. from a YAML config) without passing
  instances around.
* ``KyberControllerConfigBase`` -- the parent attrs config every
  per-controller config inherits from. Each concrete subclass pins
  ``CONTROLLER_TYPE`` as a ``ClassVar`` so the enum and the config class
  stay coupled at the source.
* ``KyberControllerManager`` -- the factory that turns a controller
  config (or a YAML dict) into a concrete ``KyberController`` instance.

The manager takes ``manipulator_model`` alongside the config because
controllers that need a Drake ``MultibodyPlant`` (diff-IK, joint-space
PID with FK lookups, etc.) build their own plant from the model
inside their constructor. Kyber itself owns no plant; that
responsibility lives with each controller, on a case-by-case basis.

Per-controller modules import the protocol / base / enum from here
and the manager imports per-controller modules lazily inside its
classmethods to keep the import graph acyclic.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, runtime_checkable

import attr

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.proprioception import Proprioception
from manor.common.exceptions import AegisConfigError
from manor.manipulators.manipulator_model import IManipulatorModel


class KyberControllerType(StrEnum):
    """
    Canonical names for Kyber controllers. Used as the ``type`` tag
    inside the ``controller_config`` block of an aegis YAML.
    """

    ZERO_VELOCITY = "zero_velocity"
    ACTION_PASSTHROUGH = "action_passthrough"


class KyberControllerYamlKey(StrEnum):
    """
    Canonical YAML field names shared across controller configs.
    """

    TYPE = "type"


@runtime_checkable
class KyberController(Protocol):
    """
    Protocol for an (action, proprioception) -> command controller.

    Concrete implementations may be purely functional (passthrough,
    zero-velocity stub) or stateful (PID with integrator state, MPC
    with internal solvers); the ``step`` interface accommodates both.
    """

    def step(self, action: Action, proprioception: Proprioception) -> Command: ...


@attr.frozen
class KyberControllerConfigBase:
    """
    Base attrs config for a Kyber controller. Concrete subclasses must
    set ``CONTROLLER_TYPE`` to the matching ``KyberControllerType``
    value; that pinning is what lets ``KyberControllerManager``
    round-trip a YAML tag through to a concrete controller without a
    parallel registry to keep in sync.
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType]


class KyberControllerManager:
    """
    Factory that turns a controller config into a ``KyberController``
    instance. ``manipulator_model`` is plumbed through so controllers
    that need a Drake plant can build one inside their constructor.

    Adding a new controller means:
      1. drop a new module under ``kyber/controllers/``,
      2. add an enum value to ``KyberControllerType``,
      3. add the dispatch branch in ``from_config`` and
         ``config_from_yaml_dict``.
    """

    @classmethod
    def from_config(
        cls,
        config: KyberControllerConfigBase,
        manipulator_model: IManipulatorModel,
    ) -> KyberController:
        """
        Build a ``KyberController`` from its config. Dispatches off
        the config's runtime type. ``manipulator_model`` is forwarded
        to controllers that ask for it (current stubs ignore it).
        """
        from manor.common.aegis.kyber.controllers.action_passthrough_controller import (
            ActionPassthroughController,
            ActionPassthroughControllerConfig,
        )
        from manor.common.aegis.kyber.controllers.zero_velocity_controller import (
            ZeroVelocityController,
            ZeroVelocityControllerConfig,
        )

        del manipulator_model  # Not needed by current controllers; reserved for plant-building ones.

        if isinstance(config, ZeroVelocityControllerConfig):
            return ZeroVelocityController(num_dof=config.num_dof)
        if isinstance(config, ActionPassthroughControllerConfig):
            return ActionPassthroughController(num_dof=config.num_dof)
        raise AegisConfigError(f"Unknown controller config type: {type(config).__name__}")

    @classmethod
    def config_from_yaml_dict(cls, raw: object) -> KyberControllerConfigBase:
        """
        Parse the ``controller_config`` block of an aegis YAML into a
        concrete ``KyberControllerConfigBase`` subclass.
        """
        from manor.common.aegis.kyber.controllers.action_passthrough_controller import (
            ActionPassthroughControllerConfig,
        )
        from manor.common.aegis.kyber.controllers.zero_velocity_controller import ZeroVelocityControllerConfig

        if not isinstance(raw, dict):
            raise AegisConfigError(f"controller_config must be a mapping; got {type(raw).__name__}")
        type_value = raw.get(KyberControllerYamlKey.TYPE)
        if not isinstance(type_value, str) or not type_value:
            raise AegisConfigError(
                f"controller_config.{KyberControllerYamlKey.TYPE} is required and must be a non-empty string"
            )
        try:
            controller_type = KyberControllerType(type_value)
        except ValueError as e:
            raise AegisConfigError(
                f"Unknown controller_config.{KyberControllerYamlKey.TYPE}: {type_value!r}; "
                f"expected one of {[t.value for t in KyberControllerType]}"
            ) from e

        body = {k: v for k, v in raw.items() if k != KyberControllerYamlKey.TYPE}
        if controller_type is KyberControllerType.ZERO_VELOCITY:
            return ZeroVelocityControllerConfig.from_yaml_dict(body)
        if controller_type is KyberControllerType.ACTION_PASSTHROUGH:
            return ActionPassthroughControllerConfig.from_yaml_dict(body)
        raise AegisConfigError(f"No config parser registered for controller type {controller_type!r}")
