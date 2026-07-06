"""
Tests for RebotB601DmModel.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from manor.common.testing_utils import run_manor_tests
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.model import (
    REBOT_B601_DM_ARM_DOF,
    REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
    REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF,
    REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M,
    REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF,
    RebotB601DmModel,
)
from manor.manipulators.rebot_b601_dm.variant import RebotB601DmVariant


def _model() -> RebotB601DmModel:
    return RebotB601DmModel(variant=RebotB601DmVariant.PARALLEL_GRIPPER)


class TestRebotB601DmModelIdentity:
    def test_implements_imanipulator_model(self) -> None:
        assert isinstance(_model(), IManipulatorModel)

    def test_get_manipulator_type(self) -> None:
        assert _model().get_manipulator_type() is ManipulatorType.REBOT_B601_DM

    def test_get_variant_round_trip(self) -> None:
        assert _model().get_variant() is RebotB601DmVariant.PARALLEL_GRIPPER


class TestRebotB601DmModelShape:
    def test_arm_dof(self) -> None:
        assert _model().get_num_dof() == REBOT_B601_DM_ARM_DOF

    def test_plant_dofs_include_gripper(self) -> None:
        m = _model()
        expected_q = REBOT_B601_DM_ARM_DOF + REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF
        assert m.get_num_positions() == expected_q
        assert m.get_num_velocities() == expected_q
        assert m.get_num_states() == 2 * expected_q

    def test_ee_dof_is_one(self) -> None:
        # The EE interface exposes a single opening width; the URDF's two prismatic finger joints are an
        # internal detail of the plant.
        assert _model().get_num_ee_dofs() == REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF


class TestRebotB601DmModelDescription:
    def test_description_filepath_exists(self) -> None:
        assert os.path.isfile(_model().get_description_filepath())

    def test_frame_names_are_canonical(self) -> None:
        m = _model()
        assert m.get_base_frame_name() == "base_link"
        assert m.get_fk_ik_frame_name() == "link_eef_tip"


class TestRebotB601DmModelEEPositionsToPlantPositions:
    def test_closed_width_is_urdf_origin(self) -> None:
        # The fingertips touch at q = (0, 0), so the closed width (zero) maps to the URDF origin.
        m = _model()
        result = m.ee_positions_to_plant_positions(np.array([REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M]))
        assert result.shape == (REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF,)
        np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-12)

    def test_open_width_matches_urdf_limits(self) -> None:
        # The published open width lands exactly at the URDF joint limits (+0.0715, +0.0715).
        m = _model()
        result = m.ee_positions_to_plant_positions(np.array([REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M]))
        np.testing.assert_allclose(result, [0.0715, 0.0715])

    def test_rejects_wrong_shape(self) -> None:
        with pytest.raises(ValueError):
            _model().ee_positions_to_plant_positions(np.array([0.04, 0.04]))


class TestRebotB601DmModelPlantPositionsToEEPositions:
    def test_urdf_origin_is_closed_width(self) -> None:
        result = _model().plant_positions_to_ee_positions(np.array([0.0, 0.0]))
        np.testing.assert_allclose(result, [REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M])

    def test_rejects_wrong_shape(self) -> None:
        with pytest.raises(ValueError):
            _model().plant_positions_to_ee_positions(np.array([0.02]))

    def test_round_trip(self) -> None:
        m = _model()
        for w in [
            REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
            REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M,
            0.05,
        ]:
            ee = np.array([w])
            np.testing.assert_allclose(m.plant_positions_to_ee_positions(m.ee_positions_to_plant_positions(ee)), ee)


class TestRebotB601DmModelEEVelocitiesToPlantVelocities:
    def test_symmetric_split(self) -> None:
        # Both fingers open outward with positive q, so width rate w_dot maps to (+w_dot/2, +w_dot/2).
        result = _model().ee_velocities_to_plant_velocities(np.array([0.04]))
        np.testing.assert_allclose(result, [0.02, 0.02])

    def test_rejects_wrong_shape(self) -> None:
        with pytest.raises(ValueError):
            _model().ee_velocities_to_plant_velocities(np.array([0.02, 0.02]))


class TestRebotB601DmModelPlantVelocitiesToEEVelocities:
    def test_sum_is_width_rate(self) -> None:
        result = _model().plant_velocities_to_ee_velocities(np.array([0.02, 0.02]))
        np.testing.assert_allclose(result, [0.04])

    def test_rejects_wrong_shape(self) -> None:
        with pytest.raises(ValueError):
            _model().plant_velocities_to_ee_velocities(np.array([0.02]))

    def test_velocity_round_trip(self) -> None:
        m = _model()
        ee_v = np.array([0.03])
        np.testing.assert_allclose(m.plant_velocities_to_ee_velocities(m.ee_velocities_to_plant_velocities(ee_v)), ee_v)


class TestRebotB601DmModelEEPositionLimits:
    def test_returns_physical_widths(self) -> None:
        lower, upper = _model().get_ee_position_limits()
        np.testing.assert_allclose(lower, [REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M])
        np.testing.assert_allclose(upper, [REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M])

    def test_lower_below_upper(self) -> None:
        lower, upper = _model().get_ee_position_limits()
        assert lower.shape == upper.shape
        assert np.all(lower <= upper)


class TestRebotB601DmModelPrimeAndRestPlantPositions:
    def test_prime_plant_positions_have_plant_size(self) -> None:
        m = _model()
        assert m.get_prime_plant_positions().shape == (m.get_num_positions(),)

    def test_rest_plant_positions_have_plant_size(self) -> None:
        m = _model()
        assert m.get_rest_plant_positions().shape == (m.get_num_positions(),)

    def test_prime_arm_block_matches_joint_configuration(self) -> None:
        np.testing.assert_allclose(
            _model().get_prime_plant_positions()[:REBOT_B601_DM_ARM_DOF],
            RebotB601DmJointConfiguration.PRIME.get_joint_positions_vector(),
        )

    def test_rest_arm_block_matches_joint_configuration(self) -> None:
        np.testing.assert_allclose(
            _model().get_rest_plant_positions()[:REBOT_B601_DM_ARM_DOF],
            RebotB601DmJointConfiguration.REST.get_joint_positions_vector(),
        )

    def test_ee_block_is_all_zeros(self) -> None:
        # The EE block at PRIME / REST is the URDF q neutral state, which is the fully closed gripper.
        m = _model()
        np.testing.assert_array_equal(
            m.get_prime_plant_positions()[REBOT_B601_DM_ARM_DOF:],
            np.zeros(REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF, dtype=np.float64),
        )
        np.testing.assert_array_equal(
            m.get_rest_plant_positions()[REBOT_B601_DM_ARM_DOF:],
            np.zeros(REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF, dtype=np.float64),
        )


if __name__ == "__main__":
    run_manor_tests()
