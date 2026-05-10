"""
Kyber controller registry and factory.

Mirrors the structure of metis/policies/policy_manager.py:

* KyberController -- the protocol every controller implementation conforms to.
* KyberControllerType -- the canonical enum used to refer to a controller by name (e.g. from a YAML
  config) without passing instances around.
* KyberControllerConfigBase -- the parent attrs config every per-controller config inherits from.
  Each concrete subclass pins CONTROLLER_TYPE as a ClassVar so the enum and the config class stay
  coupled at the source.
* KyberControllerManager -- the factory that turns a controller config (or a YAML dict) into a
  concrete KyberController instance.

The manager takes manipulator_model alongside the config because controllers that need a Drake
MultibodyPlant (diff-IK, joint-space PID with FK lookups, etc.) build their own plant from the
model inside their constructor. Kyber itself owns no plant; that responsibility lives with each
controller, on a case-by-case basis.

Per-controller modules import the protocol / base / enum from here and the manager imports
per-controller modules lazily inside its classmethods to keep the import graph acyclic.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, Self, runtime_checkable

import attr

from manor.common.definitions.action import Action
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.proprioception import Proprioception
from manor.common.exceptions import AegisConfigError
from manor.manipulators.manipulator_model import IManipulatorModel

# Tagged-union discriminator key used in the YAML body of a controller_config block. Not an attrs
# field on any per-controller config: the manager strips it before dispatching to the matching
# subclass's from_yaml_dict.
_CONTROLLER_TYPE_YAML_KEY = "type"


class KyberControllerType(StrEnum):
    """
    Canonical names for Kyber controllers. Used as the type tag inside the controller_config block
    of an aegis YAML.
    """

    ZERO_VELOCITY = "zero_velocity"
    PASSTHROUGH = "passthrough"
    IK_PASSTHROUGH = "ik_passthrough"


@runtime_checkable
class KyberController(Protocol):
    """
    Protocol for an (action, proprioception) -> joint+ee command controller.

    Concrete implementations may be purely functional (passthrough, zero-velocity stub) or stateful
    (PID with integrator state, MPC with internal solvers); the step interface accommodates both.
    """

    def step(self, action: Action, proprioception: Proprioception) -> JointEECommand: ...


@attr.frozen
class KyberControllerConfigBase:
    """
    Base attrs config for a Kyber controller. Concrete subclasses must set CONTROLLER_TYPE to the
    matching KyberControllerType value; that pinning is what lets KyberControllerManager round-trip
    a YAML tag through to a concrete controller without a parallel registry to keep in sync.

    The base also carries a from_yaml_dict that delegates to the manager. That makes it possible
    for parse_attrs_yaml to recurse into a controller_config field by type alone -- the helper sees
    KyberControllerConfigBase, calls its from_yaml_dict, and the manager dispatches to the concrete
    subclass off the type: tag.
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType]

    @classmethod
    def from_yaml_dict(cls, raw: object) -> Self:
        return KyberControllerManager.config_from_yaml_dict(raw)


class KyberControllerManager:
    """
    Factory that turns a controller config into a KyberController instance. manipulator_model is
    plumbed through so controllers that need a Drake plant can build one inside their constructor.

    Adding a new controller means:
      1. drop a new module under kyber/controllers/,
      2. add an enum value to KyberControllerType,
      3. add the dispatch branch in from_config and config_from_yaml_dict.
    """

    @classmethod
    def from_config(
        cls,
        config: KyberControllerConfigBase,
        manipulator_model: IManipulatorModel,
    ) -> KyberController:
        """
        Build a KyberController from its config. Dispatches off the config's runtime type.
        manipulator_model is forwarded to controllers that ask for it (current stubs ignore it).
        """
        from manor.common.aegis.kyber.controllers.ik_passthrough_controller import (
            IKPassthroughController,
            IKPassthroughControllerConfig,
        )
        from manor.common.aegis.kyber.controllers.passthrough_controller import (
            PassthroughController,
            PassthroughControllerConfig,
        )
        from manor.common.aegis.kyber.controllers.zero_velocity_controller import (
            ZeroVelocityController,
            ZeroVelocityControllerConfig,
        )

        if isinstance(config, ZeroVelocityControllerConfig):
            return ZeroVelocityController(num_dof=config.num_dof)
        if isinstance(config, PassthroughControllerConfig):
            return PassthroughController(num_dof=config.num_dof)
        if isinstance(config, IKPassthroughControllerConfig):
            return IKPassthroughController.build(config=config, manipulator_model=manipulator_model)
        raise AegisConfigError(f"Unknown controller config type: {type(config).__name__}")

    @classmethod
    def config_from_yaml_dict(cls, raw: object) -> KyberControllerConfigBase:
        """
        Parse the controller_config block of an aegis YAML into a concrete KyberControllerConfigBase
        subclass.
        """
        from manor.common.aegis.kyber.controllers.ik_passthrough_controller import IKPassthroughControllerConfig
        from manor.common.aegis.kyber.controllers.passthrough_controller import PassthroughControllerConfig
        from manor.common.aegis.kyber.controllers.zero_velocity_controller import ZeroVelocityControllerConfig

        if not isinstance(raw, dict):
            raise AegisConfigError(f"controller_config must be a mapping; got {type(raw).__name__}")
        type_value = raw.get(_CONTROLLER_TYPE_YAML_KEY)
        if not isinstance(type_value, str) or not type_value:
            raise AegisConfigError(
                f"controller_config.{_CONTROLLER_TYPE_YAML_KEY} is required and must be a non-empty string"
            )
        try:
            controller_type = KyberControllerType(type_value)
        except ValueError as e:
            raise AegisConfigError(
                f"Unknown controller_config.{_CONTROLLER_TYPE_YAML_KEY}: {type_value!r}; "
                f"expected one of {[t.value for t in KyberControllerType]}"
            ) from e

        body = {k: v for k, v in raw.items() if k != _CONTROLLER_TYPE_YAML_KEY}
        if controller_type is KyberControllerType.ZERO_VELOCITY:
            return ZeroVelocityControllerConfig.from_yaml_dict(body)
        if controller_type is KyberControllerType.PASSTHROUGH:
            return PassthroughControllerConfig.from_yaml_dict(body)
        if controller_type is KyberControllerType.IK_PASSTHROUGH:
            return IKPassthroughControllerConfig.from_yaml_dict(body)
        raise AegisConfigError(f"No config parser registered for controller type {controller_type!r}")
