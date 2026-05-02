"""
Tests for ``Lite6Model``.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import (
    LITE6_ARM_DOF,
    LITE6_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
    LITE6_PARALLEL_GRIPPER_EE_DOF,
    LITE6_PARALLEL_GRIPPER_OPEN_WIDTH_M,
    LITE6_PARALLEL_GRIPPER_PLANT_DOF,
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
        expected_q = LITE6_ARM_DOF + LITE6_PARALLEL_GRIPPER_PLANT_DOF
        assert m.get_num_positions() == expected_q
        assert m.get_num_velocities() == expected_q
        assert m.get_num_states() == 2 * expected_q


class TestLite6ModelEEDof:
    def test_vacuum_ee_dof_is_one(self) -> None:
        # Vacuum gripper is binary (on/off) -> a single DOF on the
        # EEPositions / EEVelocities vector.
        assert _model(Lite6Variant.VACUUM_GRIPPER).get_num_ee_dofs() == 1

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_gripper_ee_dof_is_one(self, variant: Lite6Variant) -> None:
        # The EE interface exposes a single opening width; the URDF's
        # two prismatic finger joints are an internal detail of the plant.
        assert _model(variant).get_num_ee_dofs() == LITE6_PARALLEL_GRIPPER_EE_DOF


class TestLite6ModelDescription:
    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_description_filepath_exists(self, variant: Lite6Variant) -> None:
        # The robot_models submodule ships every URDF the variants reference.
        assert os.path.isfile(_model(variant).get_description_filepath())

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_frame_names_are_lite6_canonical(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        assert m.get_base_frame_name() == "link_base"
        assert m.get_fk_ik_frame_name() == "link_eef_tip"


class TestLite6ModelEEPositionsToPlantPositions:
    def test_vacuum_returns_empty_array(self) -> None:
        m = _model(Lite6Variant.VACUUM_GRIPPER)
        result = m.ee_positions_to_plant_positions(np.array([0.0]))
        assert result.shape == (0,)

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_splits_width_with_signed_mirror(self, variant: Lite6Variant) -> None:
        # Width w maps to (+w/2, -w/2) per the URDF axis convention
        # (left finger axis +y, right finger axis +y but with the
        # opposite signed travel range).
        m = _model(variant)
        result = m.ee_positions_to_plant_positions(np.array([0.012]))
        assert result.shape == (LITE6_PARALLEL_GRIPPER_PLANT_DOF,)
        np.testing.assert_allclose(result, [+0.006, -0.006])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_open_width_matches_urdf_limits(self, variant: Lite6Variant) -> None:
        # Mapping the published OPEN width should land exactly at the
        # URDF joint limits (+0.008, -0.008).
        m = _model(variant)
        result = m.ee_positions_to_plant_positions(np.array([LITE6_PARALLEL_GRIPPER_OPEN_WIDTH_M]))
        np.testing.assert_allclose(result, [+0.008, -0.008])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_closed_width_is_origin(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        result = m.ee_positions_to_plant_positions(np.array([LITE6_PARALLEL_GRIPPER_CLOSED_WIDTH_M]))
        np.testing.assert_allclose(result, [0.0, 0.0])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_rejects_wrong_shape(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        with pytest.raises(ValueError):
            m.ee_positions_to_plant_positions(np.array([0.04, 0.04]))


class TestLite6ModelPlantPositionsToEEPositions:
    def test_vacuum_returns_zero_one_vector(self) -> None:
        m = _model(Lite6Variant.VACUUM_GRIPPER)
        result = m.plant_positions_to_ee_positions(np.zeros(0))
        assert result.shape == (1,)
        np.testing.assert_allclose(result, [0.0])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_difference_is_opening_width(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        result = m.plant_positions_to_ee_positions(np.array([+0.006, -0.006]))
        np.testing.assert_allclose(result, [0.012])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_rejects_wrong_shape(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        with pytest.raises(ValueError):
            m.plant_positions_to_ee_positions(np.array([0.02]))

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_round_trip(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        for ee in (
            np.array([LITE6_PARALLEL_GRIPPER_OPEN_WIDTH_M]),
            np.array([LITE6_PARALLEL_GRIPPER_CLOSED_WIDTH_M]),
            np.array([0.012]),
        ):
            np.testing.assert_allclose(m.plant_positions_to_ee_positions(m.ee_positions_to_plant_positions(ee)), ee)


if __name__ == "__main__":
    run_manor_tests()
