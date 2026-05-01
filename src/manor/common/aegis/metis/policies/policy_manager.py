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
from typing import ClassVar, Protocol, Self, runtime_checkable

import attr

from manor.common.definitions.action import Action
from manor.common.definitions.observation import Observation
from manor.common.exceptions import AegisConfigError

# Tagged-union discriminator key used in the YAML body of a
# ``policy_config`` block. Not an attrs field on any per-policy
# config: the manager strips it before dispatching to the matching
# subclass's ``from_yaml_dict``.
_POLICY_TYPE_YAML_KEY = "type"


class MetisPolicyType(StrEnum):
    """
    Canonical names for Metis policies. Used as the ``type`` tag inside
    the ``policy_config`` block of an aegis YAML.
    """

    IDENTITY = "identity"
    CONSTANT_JOINT_POSITIONS = "constant_joint_positions"
    CONSTANT_JOINT_VELOCITIES = "constant_joint_velocities"
    CONSTANT_CARTESIAN_POSE = "constant_cartesian_pose"
    CIRCLE_EE_VELOCITY = "circle_ee_velocity"
    GRIPPER_OPEN_CLOSE = "gripper_open_close"


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

    The base also carries a ``from_yaml_dict`` that delegates to the
    manager. That makes it possible for ``parse_attrs_yaml`` to recurse
    into a ``policy_config`` field by type alone -- the helper sees
    ``MetisPolicyConfigBase``, calls its ``from_yaml_dict``, and the
    manager dispatches to the concrete subclass off the ``type:`` tag.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType]

    @classmethod
    def from_yaml_dict(cls, raw: object) -> Self:
        return MetisPolicyManager.config_from_yaml_dict(raw)


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
        from manor.common.aegis.metis.policies.circle_ee_velocity_policy import (
            CircleEEVelocityPolicy,
            CircleEEVelocityPolicyConfig,
        )
        from manor.common.aegis.metis.policies.constant_cartesian_pose_policy import (
            ConstantCartesianPosePolicy,
            ConstantCartesianPosePolicyConfig,
        )
        from manor.common.aegis.metis.policies.constant_policies import (
            ConstantJointPositionsPolicy,
            ConstantJointPositionsPolicyConfig,
            ConstantJointVelocitiesPolicy,
            ConstantJointVelocitiesPolicyConfig,
        )
        from manor.common.aegis.metis.policies.gripper_open_close_policy import (
            GripperOpenClosePolicy,
            GripperOpenClosePolicyConfig,
        )
        from manor.common.aegis.metis.policies.identity_policy import IdentityPolicy, IdentityPolicyConfig

        if isinstance(config, IdentityPolicyConfig):
            return IdentityPolicy(num_joints=config.num_joints)
        if isinstance(config, ConstantJointPositionsPolicyConfig):
            return ConstantJointPositionsPolicy(positions=config.positions)
        if isinstance(config, ConstantJointVelocitiesPolicyConfig):
            return ConstantJointVelocitiesPolicy(velocities=config.velocities)
        if isinstance(config, ConstantCartesianPosePolicyConfig):
            return ConstantCartesianPosePolicy.from_config(config)
        if isinstance(config, CircleEEVelocityPolicyConfig):
            return CircleEEVelocityPolicy.from_config(config)
        if isinstance(config, GripperOpenClosePolicyConfig):
            return GripperOpenClosePolicy.from_config(config)
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
        from manor.common.aegis.metis.policies.circle_ee_velocity_policy import CircleEEVelocityPolicyConfig
        from manor.common.aegis.metis.policies.constant_cartesian_pose_policy import (
            ConstantCartesianPosePolicyConfig,
        )
        from manor.common.aegis.metis.policies.constant_policies import (
            ConstantJointPositionsPolicyConfig,
            ConstantJointVelocitiesPolicyConfig,
        )
        from manor.common.aegis.metis.policies.gripper_open_close_policy import GripperOpenClosePolicyConfig
        from manor.common.aegis.metis.policies.identity_policy import IdentityPolicyConfig

        if not isinstance(raw, dict):
            raise AegisConfigError(f"policy_config must be a mapping; got {type(raw).__name__}")
        type_value = raw.get(_POLICY_TYPE_YAML_KEY)
        if not isinstance(type_value, str) or not type_value:
            raise AegisConfigError(f"policy_config.{_POLICY_TYPE_YAML_KEY} is required and must be a non-empty string")
        try:
            policy_type = MetisPolicyType(type_value)
        except ValueError as e:
            raise AegisConfigError(
                f"Unknown policy_config.{_POLICY_TYPE_YAML_KEY}: {type_value!r}; "
                f"expected one of {[t.value for t in MetisPolicyType]}"
            ) from e

        body = {k: v for k, v in raw.items() if k != _POLICY_TYPE_YAML_KEY}
        if policy_type is MetisPolicyType.IDENTITY:
            return IdentityPolicyConfig.from_yaml_dict(body)
        if policy_type is MetisPolicyType.CONSTANT_JOINT_POSITIONS:
            return ConstantJointPositionsPolicyConfig.from_yaml_dict(body)
        if policy_type is MetisPolicyType.CONSTANT_JOINT_VELOCITIES:
            return ConstantJointVelocitiesPolicyConfig.from_yaml_dict(body)
        if policy_type is MetisPolicyType.CONSTANT_CARTESIAN_POSE:
            return ConstantCartesianPosePolicyConfig.from_yaml_dict(body)
        if policy_type is MetisPolicyType.CIRCLE_EE_VELOCITY:
            return CircleEEVelocityPolicyConfig.from_yaml_dict(body)
        if policy_type is MetisPolicyType.GRIPPER_OPEN_CLOSE:
            return GripperOpenClosePolicyConfig.from_yaml_dict(body)
        raise AegisConfigError(f"No config parser registered for policy type {policy_type!r}")
