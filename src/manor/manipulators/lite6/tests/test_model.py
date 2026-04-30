"""
Tests for ``Lite6Model``.
"""

from __future__ import annotations

import os

import pytest

from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import (
    LITE6_ARM_DOF,
    LITE6_PARALLEL_GRIPPER_DOF,
    Lite6Model,
)
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType


def _model(variant: Lite6Variant) -> Lite6Model:
    return Lite6Model(variant=variant)


class TestLite6ModelIdentity:
    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_implements_imanipulator_model(self, variant: Lite6Variant) -> None:
        assert isinstance(_model(variant), IManipulatorModel)

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_get_manipulator_type(self, variant: Lite6Variant) -> None:
        assert _model(variant).get_manipulator_type() is ManipulatorType.LITE6

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_get_variant_round_trip(self, variant: Lite6Variant) -> None:
        assert _model(variant).get_variant() is variant


class TestLite6ModelShape:
    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_arm_dof_constant_across_variants(self, variant: Lite6Variant) -> None:
        assert _model(variant).get_num_dof() == LITE6_ARM_DOF

    def test_vacuum_variant_has_no_extra_dofs(self) -> None:
        m = _model(Lite6Variant.VACUUM_GRIPPER)
        assert m.get_num_positions() == LITE6_ARM_DOF
        assert m.get_num_velocities() == LITE6_ARM_DOF
        assert m.get_num_states() == 2 * LITE6_ARM_DOF

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_gripper_variants_add_two_dofs(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        expected_q = LITE6_ARM_DOF + LITE6_PARALLEL_GRIPPER_DOF
        assert m.get_num_positions() == expected_q
        assert m.get_num_velocities() == expected_q
        assert m.get_num_states() == 2 * expected_q


class TestLite6ModelEEDof:
    def test_vacuum_ee_dof_is_one(self) -> None:
        # Vacuum gripper is binary (on/off) -> a single DOF on the
        # EEPositions / EEVelocities vector.
        assert _model(Lite6Variant.VACUUM_GRIPPER).get_num_ee_dofs() == 1

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_gripper_ee_dof_matches_prismatic_count(self, variant: Lite6Variant) -> None:
        assert _model(variant).get_num_ee_dofs() == LITE6_PARALLEL_GRIPPER_DOF


class TestLite6ModelDescription:
    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_description_filepath_exists(self, variant: Lite6Variant) -> None:
        # The robot_models submodule ships every URDF the variants reference.
        assert os.path.isfile(_model(variant).get_description_filepath())

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_frame_names_are_lite6_canonical(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        assert m.get_base_frame_name() == "link_base"
        assert m.get_cartesian_tip_frame_name() == "link_eef_tip"


if __name__ == "__main__":
    run_manor_tests()
