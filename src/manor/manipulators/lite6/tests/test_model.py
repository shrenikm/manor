"""
Tests for ``Lite6Model``.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.joint_configurations import Lite6JointConfiguration
from manor.manipulators.lite6.model import (
    LITE6_ARM_DOF,
    LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
    LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M,
    LITE6_PARALLEL_GRIPPER_EE_DOF,
    LITE6_PARALLEL_GRIPPER_PLANT_DOF,
    LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
    LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M,
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

    def test_normal_closed_width_is_urdf_origin(self) -> None:
        # The published NP closed width corresponds to URDF q=(0, 0)
        # (jaws as close as the normal-mounting geometry allows).
        m = _model(Lite6Variant.PARALLEL_GRIPPER_NORMAL)
        result = m.ee_positions_to_plant_positions(np.array([LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M]))
        assert result.shape == (LITE6_PARALLEL_GRIPPER_PLANT_DOF,)
        np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-12)

    def test_normal_open_width_matches_urdf_limits(self) -> None:
        # The published NP open width should land exactly at the URDF
        # joint limits (+0.008, -0.008) -- maximum physical opening.
        m = _model(Lite6Variant.PARALLEL_GRIPPER_NORMAL)
        result = m.ee_positions_to_plant_positions(np.array([LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M]))
        np.testing.assert_allclose(result, [+0.008, -0.008])

    def test_reverse_closed_width_is_urdf_origin(self) -> None:
        # The published RP closed width also corresponds to URDF
        # q=(0, 0), but the URDF link origin embeds a 27 mm gap so the
        # closed width is non-zero at the EE level.
        m = _model(Lite6Variant.PARALLEL_GRIPPER_REVERSE)
        result = m.ee_positions_to_plant_positions(np.array([LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M]))
        np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-12)

    def test_reverse_open_width_matches_urdf_limits(self) -> None:
        m = _model(Lite6Variant.PARALLEL_GRIPPER_REVERSE)
        result = m.ee_positions_to_plant_positions(np.array([LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M]))
        np.testing.assert_allclose(result, [+0.008, -0.008])

    def test_normal_and_reverse_published_widths_differ(self) -> None:
        # The two parallel-gripper variants have different physical
        # ranges because the URDF link origins differ. RP has a built-
        # in 27 mm gap that NP doesn't.
        assert LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M != LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M
        assert LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M != LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M
        # Both ranges have the same span (joint travel is shared).
        np.testing.assert_allclose(
            LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M - LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
            LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M - LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
        )

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

    def test_normal_at_urdf_origin_is_closed_width(self) -> None:
        m = _model(Lite6Variant.PARALLEL_GRIPPER_NORMAL)
        result = m.plant_positions_to_ee_positions(np.array([0.0, 0.0]))
        np.testing.assert_allclose(result, [LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M])

    def test_reverse_at_urdf_origin_is_closed_width(self) -> None:
        m = _model(Lite6Variant.PARALLEL_GRIPPER_REVERSE)
        result = m.plant_positions_to_ee_positions(np.array([0.0, 0.0]))
        np.testing.assert_allclose(result, [LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_rejects_wrong_shape(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        with pytest.raises(ValueError):
            m.plant_positions_to_ee_positions(np.array([0.02]))

    @pytest.mark.parametrize(
        "variant, widths",
        [
            (
                Lite6Variant.PARALLEL_GRIPPER_NORMAL,
                [LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M, LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M, 0.012],
            ),
            (
                Lite6Variant.PARALLEL_GRIPPER_REVERSE,
                [LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M, LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M, 0.035],
            ),
        ],
    )
    def test_round_trip(self, variant: Lite6Variant, widths: list[float]) -> None:
        m = _model(variant)
        for w in widths:
            ee = np.array([w])
            np.testing.assert_allclose(m.plant_positions_to_ee_positions(m.ee_positions_to_plant_positions(ee)), ee)


class TestLite6ModelEEVelocitiesToPlantVelocities:
    def test_vacuum_returns_empty_array(self) -> None:
        m = _model(Lite6Variant.VACUUM_GRIPPER)
        result = m.ee_velocities_to_plant_velocities(np.array([0.0]))
        assert result.shape == (0,)

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_signed_mirror(self, variant: Lite6Variant) -> None:
        # The URDF origin offset is constant in time, so the velocity
        # mapping is the same for both mountings: width rate w_dot maps
        # to (+w_dot/2, -w_dot/2).
        m = _model(variant)
        result = m.ee_velocities_to_plant_velocities(np.array([0.04]))
        np.testing.assert_allclose(result, [+0.02, -0.02])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_rejects_wrong_shape(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        with pytest.raises(ValueError):
            m.ee_velocities_to_plant_velocities(np.array([0.02, 0.02]))


class TestLite6ModelPlantVelocitiesToEEVelocities:
    def test_vacuum_returns_zero_one_vector(self) -> None:
        m = _model(Lite6Variant.VACUUM_GRIPPER)
        result = m.plant_velocities_to_ee_velocities(np.zeros(0))
        assert result.shape == (1,)
        np.testing.assert_allclose(result, [0.0])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_parallel_difference_is_width_rate(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        result = m.plant_velocities_to_ee_velocities(np.array([+0.02, -0.02]))
        np.testing.assert_allclose(result, [0.04])

    @pytest.mark.parametrize("variant", [Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE])
    def test_velocity_round_trip(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        ee_v = np.array([0.03])
        np.testing.assert_allclose(m.plant_velocities_to_ee_velocities(m.ee_velocities_to_plant_velocities(ee_v)), ee_v)


class TestLite6ModelEEPositionLimits:
    def test_vacuum_returns_zero_one_range(self) -> None:
        m = _model(Lite6Variant.VACUUM_GRIPPER)
        lower, upper = m.get_ee_position_limits()
        np.testing.assert_array_equal(lower, [0.0])
        np.testing.assert_array_equal(upper, [1.0])

    def test_normal_parallel_returns_physical_widths(self) -> None:
        m = _model(Lite6Variant.PARALLEL_GRIPPER_NORMAL)
        lower, upper = m.get_ee_position_limits()
        np.testing.assert_allclose(lower, [LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M])
        np.testing.assert_allclose(upper, [LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M])

    def test_reverse_parallel_returns_physical_widths(self) -> None:
        m = _model(Lite6Variant.PARALLEL_GRIPPER_REVERSE)
        lower, upper = m.get_ee_position_limits()
        np.testing.assert_allclose(lower, [LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M])
        np.testing.assert_allclose(upper, [LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M])

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_lower_below_upper(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        lower, upper = m.get_ee_position_limits()
        assert lower.shape == upper.shape
        assert np.all(lower <= upper)


class TestLite6ModelPrimeAndRestPlantPositions:
    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_prime_plant_positions_have_plant_size(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        assert m.get_prime_plant_positions().shape == (m.get_num_positions(),)

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_rest_plant_positions_have_plant_size(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        assert m.get_rest_plant_positions().shape == (m.get_num_positions(),)

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_prime_arm_block_matches_lite6_joint_configuration(self, variant: Lite6Variant) -> None:
        m = _model(variant)
        np.testing.assert_allclose(
            m.get_prime_plant_positions()[:LITE6_ARM_DOF],
            Lite6JointConfiguration.PRIME.get_joint_positions_vector(),
        )

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_rest_arm_block_matches_lite6_joint_configuration(self, variant: Lite6Variant) -> None:
        # REST happens to be all-zero on the Lite6, but the test asserts via the configuration's
        # own vector so the property remains true if the rest pose is ever retuned.
        m = _model(variant)
        np.testing.assert_allclose(
            m.get_rest_plant_positions()[:LITE6_ARM_DOF],
            Lite6JointConfiguration.REST.get_joint_positions_vector(),
        )

    @pytest.mark.parametrize("variant", list(Lite6Variant))
    def test_ee_block_is_all_zeros(self, variant: Lite6Variant) -> None:
        # Vacuum: 0-length EE block (no actuated joints). Parallel: 2-length, both at the URDF q
        # neutral state which is the variant's "closed" width.
        m = _model(variant)
        np.testing.assert_array_equal(
            m.get_prime_plant_positions()[LITE6_ARM_DOF:],
            np.zeros(m.get_num_positions() - LITE6_ARM_DOF, dtype=np.float64),
        )
        np.testing.assert_array_equal(
            m.get_rest_plant_positions()[LITE6_ARM_DOF:],
            np.zeros(m.get_num_positions() - LITE6_ARM_DOF, dtype=np.float64),
        )


if __name__ == "__main__":
    run_manor_tests()
