"""
Tests for RebotB601DmVariant.
"""

from __future__ import annotations

import pytest

from manor.common.testing_utils import run_manor_tests
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import IManipulatorVariant
from manor.manipulators.rebot_b601_dm.variant import RebotB601DmVariant


class TestRebotB601DmVariant:
    def test_is_instance_of_imanipulator_variant(self) -> None:
        assert issubclass(RebotB601DmVariant, IManipulatorVariant)

    @pytest.mark.parametrize("variant", list(RebotB601DmVariant))
    def test_get_manipulator_type_returns_rebot_b601_dm(self, variant: RebotB601DmVariant) -> None:
        assert variant.get_manipulator_type() is ManipulatorType.REBOT_B601_DM

    def test_str_enum_values_are_stable(self) -> None:
        # The string values are referenced from the URDF mapping table and from any external config;
        # treat them as part of the API.
        assert RebotB601DmVariant.PARALLEL_GRIPPER.value == "parallel_gripper"


if __name__ == "__main__":
    run_manor_tests()
