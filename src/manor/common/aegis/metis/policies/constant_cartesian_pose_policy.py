"""
ConstantCartesianPosePolicy: emits a fixed Cartesian pose target on
every step. Pairs with IKPassthroughController to drive the arm to a
fixed task-space goal regardless of the observation.

The orientation defaults to the identity quaternion (1, 0, 0, 0) so the
policy is usable with translation-only YAML configs.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml, require_number
from manor.common.custom_types import NpVector3f64, NpVector4f64
from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError


def _parse_translation(value: object, context: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        raise AegisConfigError(f"{context} must be a list of 3 numbers")
    coerced = [require_number(item, f"{context}[{i}]") for i, item in enumerate(value)]
    return np.array(coerced, dtype=np.float64)


def _parse_orientation(value: object, context: str) -> np.ndarray:
    if value is None:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if not isinstance(value, list) or len(value) != 4:
        raise AegisConfigError(f"{context} must be a list of 4 numbers (quaternion w, x, y, z)")
    coerced = [require_number(item, f"{context}[{i}]") for i, item in enumerate(value)]
    return np.array(coerced, dtype=np.float64)


@attr.frozen
class ConstantCartesianPosePolicyConfig(MetisPolicyConfigBase):
    """
    Config for ConstantCartesianPosePolicy.

    translation is the (x, y, z) target in the manipulator's base /
    world frame. orientation is the (w, x, y, z) unit quaternion target;
    omitted in YAML defaults to identity.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.CONSTANT_CARTESIAN_POSE

    translation: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    orientation: NpVector4f64 = attr.field(
        factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        eq=attr.cmp_using(eq=np.array_equal),
    )

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(
            **parse_attrs_yaml(
                cls,
                d,
                "ConstantCartesianPosePolicyConfig",
                custom_parsers={
                    "translation": _parse_translation,
                    "orientation": _parse_orientation,
                },
            )
        )


@attr.frozen
class ConstantCartesianPosePolicy:
    """
    Always emit the same CartesianCommand with a CartesianPose target.
    """

    translation: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    orientation: NpVector4f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))

    @classmethod
    def from_config(cls, config: ConstantCartesianPosePolicyConfig) -> Self:
        return cls(translation=config.translation, orientation=config.orientation)

    def step(self, observation: Observation) -> Action:
        del observation
        header = TimestampHeader.from_system_time()
        return Action(
            header=header,
            cartesian_command=CartesianCommand(
                header=header,
                cartesian_pose=CartesianPose(
                    header=header,
                    translation=np.asarray(self.translation, dtype=np.float64).copy(),
                    orientation=np.asarray(self.orientation, dtype=np.float64).copy(),
                ),
            ),
        )
