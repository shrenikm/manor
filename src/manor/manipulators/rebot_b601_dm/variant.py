"""
reBot B601 DM variant enum.

The arm ships as a single trim: 6-DOF Damiao-motor arm with the stock actuated parallel gripper. A
single PARALLEL_GRIPPER variant keeps the (type, variant) interface uniform with other families and
leaves room for future trims (alternative end-effectors, teleop leader arm, etc).
"""

from __future__ import annotations

from typing import override

from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import IManipulatorVariant, register_manipulator_variant


@register_manipulator_variant(ManipulatorType.REBOT_B601_DM)
class RebotB601DmVariant(IManipulatorVariant):
    """
    Concrete trim variants of the Seeed reBot B601 DM.
    """

    PARALLEL_GRIPPER = "parallel_gripper"

    @override
    def get_manipulator_type(self) -> ManipulatorType:
        return ManipulatorType.REBOT_B601_DM
