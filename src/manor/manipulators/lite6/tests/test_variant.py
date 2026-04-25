"""
Tests for ``Lite6Variant``.
"""

from __future__ import annotations

import pytest

from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import IManipulatorVariant


class TestLite6Variant:
    def test_is_instance_of_imanipulator_variant(self) -> None:
        assert issubclass(Lite6Variant, IManipulatorVariant)

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_get_manipulator_type_returns_lite6(self, variant: Lite6Variant) -> None:
        assert variant.get_manipulator_type() is ManipulatorType.LITE6

    def test_str_enum_values_are_stable(self) -> None:
        # The string values are referenced from the URDF mapping table
        # and from any external config; treat them as part of the API.
        assert Lite6Variant.VACUUM_GRIPPER.value == "vacuum_gripper"
        assert Lite6Variant.PARALLEL_GRIPPER.value == "parallel_gripper"
        assert Lite6Variant.PARALLEL_GRIPPER_REVERSE.value == "parallel_gripper_reverse"


if __name__ == "__main__":
    run_manor_tests()
