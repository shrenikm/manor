"""
Lite6 variant enum.

Three trims are supported in Manor:
    - VACUUM_GRIPPER           : Lite6 with vacuum gripper end-effector
    - PARALLEL_GRIPPER_NORMAL  : Lite6 with actuated parallel gripper, normal mounting
    - PARALLEL_GRIPPER_REVERSE : Lite6 with actuated parallel gripper, reverse mounting

Vacuum-only and unactuated parallel-gripper trims that existed in the
deprecated codebase are intentionally dropped: Manor only targets
actuated-gripper variants.
"""

from __future__ import annotations

from typing import override

from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import IManipulatorVariant, register_manipulator_variant


@register_manipulator_variant(ManipulatorType.LITE6)
class Lite6Variant(IManipulatorVariant):
    """
    Concrete trim variants of the Ufactory Lite6.
    """

    VACUUM_GRIPPER = "vacuum_gripper"
    PARALLEL_GRIPPER_NORMAL = "parallel_gripper_normal"
    PARALLEL_GRIPPER_REVERSE = "parallel_gripper_reverse"

    @override
    def get_manipulator_type(self) -> ManipulatorType:
        return ManipulatorType.LITE6
