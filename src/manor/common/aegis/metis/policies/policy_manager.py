"""
Metis policy registry and factory.

This module owns the public surface of the Metis policy abstraction:

* ``MetisPolicy`` -- the protocol every policy implementation conforms to.
* ``MetisPolicyType`` -- the canonical enum used to refer to a policy
  by name (e.g. from a YAML config) without passing instances around.
* ``MetisPolicyConfigBase`` -- the parent attrs config every per-policy
  config inherits from. Each concrete subclass pins ``POLICY_TYPE`` as
  a ``ClassVar`` so the enum and the config class are coupled at the
  source.
* ``MetisPolicyManager`` -- the factory that turns a policy config (or
  a YAML dict) into a concrete ``MetisPolicy`` instance.

Per-policy modules import the protocol / base / enum from here and
register their own concrete config + policy classes; the manager
imports them lazily inside its classmethods to keep the import graph
acyclic.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, runtime_checkable

import attr

from manor.common.definitions.action import Action
from manor.common.definitions.observation import Observation
from manor.common.exceptions import AegisConfigError


class MetisPolicyType(StrEnum):
    """
    Canonical names for Metis policies. Used as the ``type`` tag inside
    the ``policy_config`` block of an aegis YAML.
    """

    ZERO_VELOCITY = "zero_velocity"
    IDENTITY = "identity"


class MetisPolicyYamlKey(StrEnum):
    """
    Canonical YAML field names shared across policy configs.
    """

    TYPE = "type"


@runtime_checkable
class MetisPolicy(Protocol):
    """
    Protocol for an observation-to-action policy.

    Concrete implementations may be purely functional (classical
    planners, trajopt) or stateful (learned policies with internal
    recurrence); the ``step`` interface accommodates both.
    """

    def step(self, observation: Observation) -> Action: ...


@attr.frozen
class MetisPolicyConfigBase:
    """
    Base attrs config for a Metis policy. Concrete subclasses must set
    ``POLICY_TYPE`` to the matching ``MetisPolicyType`` value; that
    pinning is what lets ``MetisPolicyManager`` round-trip a YAML tag
    through to a concrete policy without a parallel registry to keep
    in sync.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType]


class MetisPolicyManager:
    """
    Factory that turns a policy config into a ``MetisPolicy`` instance.

    The manager is the single place that knows about every concrete
    policy + policy-config pair. Adding a new policy means:

      1. drop a new module under ``metis/policies/``,
      2. add an enum value to ``MetisPolicyType``,
      3. add the dispatch branch in ``from_config`` and
         ``config_from_yaml_dict``.
    """

    @classmethod
    def from_config(cls, config: MetisPolicyConfigBase) -> MetisPolicy:
        """
        Build a ``MetisPolicy`` from its config. Dispatches off the
        config's runtime type (which is itself anchored to
        ``POLICY_TYPE``).
        """
        from manor.common.aegis.metis.policies.identity_policy import IdentityPolicy, IdentityPolicyConfig
        from manor.common.aegis.metis.policies.zero_velocity_policy import (
            ZeroVelocityPolicy,
            ZeroVelocityPolicyConfig,
        )

        if isinstance(config, ZeroVelocityPolicyConfig):
            return ZeroVelocityPolicy(num_joints=config.num_joints)
        if isinstance(config, IdentityPolicyConfig):
            return IdentityPolicy(num_joints=config.num_joints)
        raise AegisConfigError(f"Unknown policy config type: {type(config).__name__}")

    @classmethod
    def config_from_yaml_dict(cls, raw: object) -> MetisPolicyConfigBase:
        """
        Parse the ``policy_config`` block of an aegis YAML into a
        concrete ``MetisPolicyConfigBase`` subclass. The block must
        carry a ``type`` key matching one of the ``MetisPolicyType``
        values; the rest of the block is forwarded to that subclass's
        ``from_yaml_dict``.
        """
        from manor.common.aegis.metis.policies.identity_policy import IdentityPolicyConfig
        from manor.common.aegis.metis.policies.zero_velocity_policy import ZeroVelocityPolicyConfig

        if not isinstance(raw, dict):
            raise AegisConfigError(f"policy_config must be a mapping; got {type(raw).__name__}")
        type_value = raw.get(MetisPolicyYamlKey.TYPE)
        if not isinstance(type_value, str) or not type_value:
            raise AegisConfigError(
                f"policy_config.{MetisPolicyYamlKey.TYPE} is required and must be a non-empty string"
            )
        try:
            policy_type = MetisPolicyType(type_value)
        except ValueError as e:
            raise AegisConfigError(
                f"Unknown policy_config.{MetisPolicyYamlKey.TYPE}: {type_value!r}; "
                f"expected one of {[t.value for t in MetisPolicyType]}"
            ) from e

        body = {k: v for k, v in raw.items() if k != MetisPolicyYamlKey.TYPE}
        if policy_type is MetisPolicyType.ZERO_VELOCITY:
            return ZeroVelocityPolicyConfig.from_yaml_dict(body)
        if policy_type is MetisPolicyType.IDENTITY:
            return IdentityPolicyConfig.from_yaml_dict(body)
        raise AegisConfigError(f"No config parser registered for policy type {policy_type!r}")
