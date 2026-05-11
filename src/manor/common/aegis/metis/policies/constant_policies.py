"""
Constant policies: emit a fixed Action on every step, irrespective
of the observation.

Useful as sim sanity checks. ConstantJointPositionsPolicy publishes a
fixed JointPositions target -- the simplest non-trivial policy that
exercises the metis -> kyber -> talos -> gaia path.
ConstantJointVelocitiesPolicy is the velocity-space analogue and
covers the previous zero-velocity bring-up default as the special
case where every entry is 0.

The action width is fixed at construction; these policies do not
resize themselves off proprioception, since the whole point is to
publish a constant target.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml, require_number
from manor.common.custom_types import JointPositionsVector, JointVelocitiesVector
from manor.common.definitions.action import Action
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError


def _parse_float_vector(value: object, context: str) -> np.ndarray:
    if not isinstance(value, list) or not value:
        raise AegisConfigError(f"{context} must be a non-empty list of numbers")
    coerced = [require_number(item, f"{context}[{i}]") for i, item in enumerate(value)]
    return np.array(coerced, dtype=np.float64)


@attr.frozen
class ConstantJointPositionsPolicyConfig(MetisPolicyConfigBase):
    """
    Config for ConstantJointPositionsPolicy. positions is the constant
    joint-position target the policy emits on every step.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.CONSTANT_JOINT_POSITIONS

    positions: JointPositionsVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(
            **parse_attrs_yaml(
                cls,
                d,
                "ConstantJointPositionsPolicyConfig",
                custom_parsers={"positions": _parse_float_vector},
            )
        )


@attr.frozen
class ConstantJointPositionsPolicy:
    """
    Always emit the same JointPositions action.
    """

    positions: JointPositionsVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    def step(self, observation: Observation) -> Action:
        header = TimestampHeader.from_system_time()
        return Action(
            header=header,
            joint_command=JointCommand(
                header=header,
                joint_positions=JointPositions(
                    header=header,
                    positions=np.asarray(self.positions, dtype=np.float64).copy(),
                ),
            ),
        )


@attr.frozen
class ConstantJointVelocitiesPolicyConfig(MetisPolicyConfigBase):
    """
    Config for ConstantJointVelocitiesPolicy. velocities is the constant
    joint-velocity target the policy emits on every step. An all-zero
    vector is the safe bring-up default.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.CONSTANT_JOINT_VELOCITIES

    velocities: JointVelocitiesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(
            **parse_attrs_yaml(
                cls,
                d,
                "ConstantJointVelocitiesPolicyConfig",
                custom_parsers={"velocities": _parse_float_vector},
            )
        )


@attr.frozen
class ConstantJointVelocitiesPolicy:
    """
    Always emit the same JointVelocities action. An all-zero vector is
    equivalent to the previous zero-velocity bring-up default.
    """

    velocities: JointVelocitiesVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    def step(self, observation: Observation) -> Action:
        header = TimestampHeader.from_system_time()
        return Action(
            header=header,
            joint_command=JointCommand(
                header=header,
                joint_velocities=JointVelocities(
                    header=header,
                    velocities=np.asarray(self.velocities, dtype=np.float64).copy(),
                ),
            ),
        )
