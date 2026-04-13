import os

import numpy as np
import pytest

from manr.common.testing_utils import execute_pytest_file
from manr.lite6.utils.lite6_model_utils import (
    LITE6_NP_GRIPPER_CLOSED_POSITIONS,
    LITE6_NP_GRIPPER_CLOSED_VELOCITIES,
    LITE6_NP_GRIPPER_OPEN_POSITIONS,
    LITE6_NP_GRIPPER_OPEN_VELOCITIES,
    LITE6_RP_GRIPPER_CLOSED_POSITIONS,
    LITE6_RP_GRIPPER_CLOSED_VELOCITIES,
    LITE6_RP_GRIPPER_OPEN_POSITIONS,
    LITE6_RP_GRIPPER_OPEN_VELOCITIES,
    Lite6GripperStatus,
    Lite6ModelGroups,
    Lite6ModelType,
    add_gripper_status_to_lite6_state_vector,
    add_joint_positions_vector_to_lite6_state_vector,
    add_joint_velocities_vector_to_lite6_state_vector,
    create_lite6_state_vector,
    get_default_lite6_joint_positions_vector,
    get_drake_lite6_urdf_path,
    get_gripper_positions_from_lite6_state_vector,
    get_gripper_status_from_lite6_state_vector,
    get_gripper_velocities_from_lite6_state_vector,
    get_joint_positions_vector_from_lite6_state_vector,
    get_joint_velocities_vector_from_lite6_state_vector,
    get_lite6_num_actuators,
    get_lite6_num_positions,
    get_lite6_num_states,
    get_lite6_num_velocities,
    get_lite6_table_urdf_lite6_position_frame_name,
    get_lite6_table_urdf_path,
    get_lite6_urdf_base_frame_name,
    get_lite6_urdf_eef_tip_frame_name,
    get_parallel_gripper_positions,
    get_parallel_gripper_velocities,
    get_joint_positions_vector_from_lite6_state_vector,
    get_unactuated_parallel_gripper_counterpart,
    get_velocities_from_lite6_state,
)


def test_lite6_model_groups() -> None:
    assert Lite6ModelType.NP_GRIPPER in Lite6ModelGroups.LITE6_GRIPPER_MODELS
    assert Lite6ModelType.RP_GRIPPER in Lite6ModelGroups.LITE6_GRIPPER_MODELS
    assert Lite6ModelType.V_GRIPPER in Lite6ModelGroups.LITE6_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_ANP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITH_ARP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITH_UNP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITH_URP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITH_V_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS
    assert Lite6ModelType.ROBOT_WITHOUT_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_MODELS

    assert Lite6ModelType.ROBOT_WITH_ANP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_ARP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_UNP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_URP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_V_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_ANP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_ARP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_UNP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_URP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_PARALLEL_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_V_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_VACUUM_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_ANP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_ACTUATED_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_ARP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_ACTUATED_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_ANP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_ACTUATED_PARALLEL_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_ARP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_ACTUATED_PARALLEL_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_UNP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_PARALLEL_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_URP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_PARALLEL_GRIPPER_MODELS

    assert Lite6ModelType.ROBOT_WITH_UNP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_URP_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITH_V_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_GRIPPER_MODELS
    assert Lite6ModelType.ROBOT_WITHOUT_GRIPPER in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_GRIPPER_MODELS


def test_get_drake_lite6_urdf_path() -> None:
    """
    Test that each a valid and existing model path can be obtained
    for each lite6 model type.
    """
    for lite6_model_type in Lite6ModelType:
        urdf_path = get_drake_lite6_urdf_path(
            lite6_model_type=lite6_model_type,
        )
        print(urdf_path)
        assert os.path.isfile(urdf_path)


def test_get_lite6_table_urdf_path() -> None:
    """
    Test that we can retrieve the path to the lite6 table urdf.
    """
    assert os.path.isfile(get_lite6_table_urdf_path())


def test_lite6_urdf_base_frame_name() -> None:
    for lite6_model_type in Lite6ModelType:
        base_frame_name = get_lite6_urdf_base_frame_name(
            lite6_model_type=lite6_model_type,
        )
        assert isinstance(base_frame_name, str)


def test_get_lite6_urdf_eef_tip_frame_name() -> None:
    valid_model_types = [
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
    ]
    for lite6_model_type in valid_model_types:
        assert isinstance(get_lite6_urdf_eef_tip_frame_name(lite6_model_type=lite6_model_type), str)

    for lite6_model_type in set(Lite6ModelType) - set(valid_model_types):
        with pytest.raises(AssertionError):
            get_lite6_urdf_eef_tip_frame_name(lite6_model_type=lite6_model_type)


def test_get_lite6_table_urdf_lite6_position_frame_name() -> None:
    assert get_lite6_table_urdf_lite6_position_frame_name() == "link_lite6_position"


def test_get_unactuated_parallel_gripper_counterpart() -> None:
    for lite6_model_type in [
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
    ]:
        with pytest.raises(AssertionError):
            get_unactuated_parallel_gripper_counterpart(lite6_model_type=lite6_model_type)
    assert (
        get_unactuated_parallel_gripper_counterpart(
            lite6_model_type=Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        )
        == Lite6ModelType.ROBOT_WITH_UNP_GRIPPER
    )

    assert (
        get_unactuated_parallel_gripper_counterpart(
            lite6_model_type=Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
        )
        == Lite6ModelType.ROBOT_WITH_URP_GRIPPER
    )


def test_get_lite6_num_actuators() -> None:
    assert get_lite6_num_actuators(Lite6ModelType.NP_GRIPPER) == 2
    assert get_lite6_num_actuators(Lite6ModelType.RP_GRIPPER) == 2
    assert get_lite6_num_actuators(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 6
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_ANP_GRIPPER) == 8
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_ARP_GRIPPER) == 8
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_UNP_GRIPPER) == 6
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_URP_GRIPPER) == 6
    assert get_lite6_num_actuators(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 6


def test_get_lite6_num_positions() -> None:
    assert get_lite6_num_positions(Lite6ModelType.NP_GRIPPER) == 2
    assert get_lite6_num_positions(Lite6ModelType.RP_GRIPPER) == 2
    assert get_lite6_num_positions(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 6
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_ANP_GRIPPER) == 8
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_ARP_GRIPPER) == 8
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_UNP_GRIPPER) == 6
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_URP_GRIPPER) == 6
    assert get_lite6_num_positions(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 6


def test_get_lite6_num_velocities() -> None:
    assert get_lite6_num_velocities(Lite6ModelType.NP_GRIPPER) == 2
    assert get_lite6_num_velocities(Lite6ModelType.RP_GRIPPER) == 2
    assert get_lite6_num_velocities(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 6
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_ANP_GRIPPER) == 8
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_ARP_GRIPPER) == 8
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_UNP_GRIPPER) == 6
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_URP_GRIPPER) == 6
    assert get_lite6_num_velocities(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 6


def test_get_lite6_num_states() -> None:
    assert get_lite6_num_states(Lite6ModelType.NP_GRIPPER) == 4
    assert get_lite6_num_states(Lite6ModelType.RP_GRIPPER) == 4
    assert get_lite6_num_states(Lite6ModelType.V_GRIPPER) == 0
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITHOUT_GRIPPER) == 12
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_ANP_GRIPPER) == 16
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_ARP_GRIPPER) == 16
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_UNP_GRIPPER) == 12
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_URP_GRIPPER) == 12
    assert get_lite6_num_states(Lite6ModelType.ROBOT_WITH_V_GRIPPER) == 12


def test_add_joint_positions_to_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            add_joint_positions_vector_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
                joint_positions_vector=np.zeros(6),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector_initial = np.arange(0, 16, dtype=np.float64)
        state_vector = np.copy(state_vector_initial)
        positions_vector = np.arange(6, 0, -1, dtype=np.float64)

        # With invalid positions vector
        with pytest.raises(AssertionError):
            add_joint_positions_vector_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=state_vector,
                joint_positions_vector=np.zeros(5),
            )

        new_state_vector = add_joint_positions_vector_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            joint_positions_vector=positions_vector,
        )
        # No mutation test
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                6.0,
                5.0,
                4.0,
                3.0,
                2.0,
                1.0,
                6.0,
                7.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0,
                13.0,
                14.0,
                15.0,
            ],
        )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        state_vector_initial = np.arange(0, 12, dtype=np.float64)
        state_vector = np.copy(state_vector_initial)
        positions_vector = np.arange(6, 0, -1, dtype=np.float64)

        # With invalid positions vector
        with pytest.raises(AssertionError):
            add_joint_positions_vector_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=state_vector,
                joint_positions_vector=np.zeros(5),
            )

        new_state_vector = add_joint_positions_vector_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            joint_positions_vector=positions_vector,
        )
        # No mutation test
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                6.0,
                5.0,
                4.0,
                3.0,
                2.0,
                1.0,
                6.0,
                7.0,
                8.0,
                9.0,
                10.0,
                11.0,
            ],
        )


def test_add_joint_velocities_to_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            add_joint_velocities_vector_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
                joint_velocities_vector=np.zeros(6),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector_initial = np.arange(0, 16, dtype=np.float64)
        state_vector = np.copy(state_vector_initial)
        velocities_vector = np.arange(6, 0, -1, dtype=np.float64)

        # With invalid positions vector
        with pytest.raises(AssertionError):
            add_joint_velocities_vector_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=state_vector,
                joint_velocities_vector=np.zeros(5),
            )

        new_state_vector = add_joint_velocities_vector_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            joint_velocities_vector=velocities_vector,
        )
        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                7.0,
                6.0,
                5.0,
                4.0,
                3.0,
                2.0,
                1.0,
                14.0,
                15.0,
            ],
        )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        state_vector_initial = np.arange(0, 12, dtype=np.float64)
        state_vector = np.copy(state_vector_initial)
        velocities_vector = np.arange(6, 0, -1, dtype=np.float64)

        # With invalid positions vector
        with pytest.raises(AssertionError):
            add_joint_velocities_vector_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=state_vector,
                joint_velocities_vector=np.zeros(5),
            )

        new_state_vector = add_joint_velocities_vector_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            joint_velocities_vector=velocities_vector,
        )
        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                5.0,
                4.0,
                3.0,
                2.0,
                1.0,
            ],
        )


def test_get_parallel_gripper_positions() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_parallel_gripper_positions(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.CLOSED,
            )

    for lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_WITH_ACTUATED_PARALLEL_GRIPPER_MODELS:
        # Neutral status gives (0, 0) positions.
        np.testing.assert_array_equal(
            get_parallel_gripper_positions(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.NEUTRAL,
            ),
            (0.0, 0.0),
        )

        # Open position.
        if lite6_model_type == Lite6ModelType.ROBOT_WITH_ANP_GRIPPER:
            expected_gp = LITE6_NP_GRIPPER_OPEN_POSITIONS
        else:
            expected_gp = LITE6_RP_GRIPPER_OPEN_POSITIONS
        np.testing.assert_array_equal(
            get_parallel_gripper_positions(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.OPEN,
            ),
            expected_gp,
        )

        # Closed position.
        if lite6_model_type == Lite6ModelType.ROBOT_WITH_ANP_GRIPPER:
            expected_gp = LITE6_NP_GRIPPER_CLOSED_POSITIONS
        else:
            expected_gp = LITE6_RP_GRIPPER_CLOSED_POSITIONS
        np.testing.assert_array_equal(
            get_parallel_gripper_positions(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.CLOSED,
            ),
            expected_gp,
        )


def test_get_parallel_gripper_velocities() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_parallel_gripper_velocities(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.CLOSED,
            )

    for lite6_model_type in Lite6ModelGroups.LITE6_ROBOT_WITH_ACTUATED_PARALLEL_GRIPPER_MODELS:
        # Neutral status gives (0, 0) velocities.
        np.testing.assert_array_equal(
            get_parallel_gripper_velocities(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.NEUTRAL,
            ),
            (0.0, 0.0),
        )

        # Open position.
        if lite6_model_type == Lite6ModelType.ROBOT_WITH_ANP_GRIPPER:
            expected_gv = LITE6_NP_GRIPPER_OPEN_VELOCITIES
        else:
            expected_gv = LITE6_RP_GRIPPER_OPEN_VELOCITIES
        np.testing.assert_array_equal(
            get_parallel_gripper_velocities(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.OPEN,
            ),
            expected_gv,
        )

        # Closed position.
        if lite6_model_type == Lite6ModelType.ROBOT_WITH_ANP_GRIPPER:
            expected_gv = LITE6_NP_GRIPPER_CLOSED_VELOCITIES
        else:
            expected_gv = LITE6_RP_GRIPPER_CLOSED_VELOCITIES
        np.testing.assert_array_equal(
            get_parallel_gripper_velocities(
                lite6_model_type=lite6_model_type,
                lite6_gripper_status=Lite6GripperStatus.CLOSED,
            ),
            expected_gv,
        )


def test_add_gripper_status_to_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            add_gripper_status_to_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
                lite6_gripper_status=Lite6GripperStatus.CLOSED,
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector_initial = np.arange(0, 16, dtype=np.float64)
        state_vector = np.copy(state_vector_initial)

        # For neutral, the gripper positions must be zero.
        new_state_vector = add_gripper_status_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            lite6_gripper_status=Lite6GripperStatus.NEUTRAL,
        )
        np.testing.assert_array_equal(
            new_state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                0.0,
                0.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0,
                13.0,
                0.0,
                0.0,
            ],
        )

        lite6_gripper_status = Lite6GripperStatus.CLOSED
        new_state_vector = add_gripper_status_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            lite6_gripper_status=lite6_gripper_status,
        )

        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                0.0,
                0.0,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0,
                13.0,
                -0.05,
                0.05,
            ],
        )

        lite6_gripper_status = Lite6GripperStatus.OPEN
        new_state_vector = add_gripper_status_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            lite6_gripper_status=lite6_gripper_status,
        )
        # No mutation test.
        np.testing.assert_array_equal(state_vector_initial, state_vector)
        # Test values of the new state vector.
        np.testing.assert_array_equal(
            new_state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                0.008,
                -0.008,
                8.0,
                9.0,
                10.0,
                11.0,
                12.0,
                13.0,
                0.05,
                -0.05,
            ],
        )


def test_create_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            create_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                joint_positions_vector=np.zeros(6),
                joint_velocities_vector=np.zeros(6),
                lite6_gripper_status=Lite6GripperStatus.CLOSED,
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        positions_vector = np.arange(0, 6, dtype=np.float64)
        velocities_vector = np.arange(6, 0, -1, dtype=np.float64)

        lite6_gripper_status = Lite6GripperStatus.CLOSED
        state_vector = create_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            joint_positions_vector=positions_vector,
            joint_velocities_vector=velocities_vector,
            lite6_gripper_status=lite6_gripper_status,
        )
        np.testing.assert_array_equal(
            state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                0.0,
                0.0,
                6.0,
                5.0,
                4.0,
                3.0,
                2.0,
                1.0,
                -0.05,
                0.05,
            ],
        )

        lite6_gripper_status = Lite6GripperStatus.OPEN
        state_vector = create_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            joint_positions_vector=positions_vector,
            joint_velocities_vector=velocities_vector,
            lite6_gripper_status=lite6_gripper_status,
        )
        np.testing.assert_array_equal(
            state_vector,
            [
                0.0,
                1.0,
                2.0,
                3.0,
                4.0,
                5.0,
                0.008,
                -0.008,
                6.0,
                5.0,
                4.0,
                3.0,
                2.0,
                1.0,
                0.05,
                -0.05,
            ],
        )


def test_get_joint_positions_from_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_joint_positions_vector_from_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    # For robots with no gripper actuation.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        state_vector = np.arange(0, 12, dtype=np.float64)

        positions_vector = get_joint_positions_vector_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        )

    # For parallel gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        positions_vector = get_joint_positions_vector_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        )


def test_get_joint_velocities_from_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_joint_velocities_vector_from_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    # For robots with no gripper actuation.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        state_vector = np.arange(0, 12, dtype=np.float64)

        positions_vector = get_joint_velocities_vector_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [6.0, 7.0, 8.0, 9.0, 10.0, 11.0],
        )

    # For parallel gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        positions_vector = get_joint_velocities_vector_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [8.0, 9.0, 10.0, 11.0, 12.0, 13.0],
        )


def test_get_positions_from_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_joint_positions_vector_from_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    # For non actuated gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        state_vector = np.arange(0, 12, dtype=np.float64)

        positions_vector = get_joint_positions_vector_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        )

    # For parallel gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        positions_vector = get_joint_positions_vector_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        )


def test_get_velocities_from_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_velocities_from_lite6_state(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    # For non actuated gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        state_vector = np.arange(0, 12, dtype=np.float64)

        positions_vector = get_velocities_from_lite6_state(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [6.0, 7.0, 8.0, 9.0, 10.0, 11.0],
        )

    # For parallel gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        positions_vector = get_velocities_from_lite6_state(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        # Test values of the new state.
        np.testing.assert_array_equal(
            positions_vector,
            [8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
        )


def test_get_gripper_positions_from_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_gripper_positions_from_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        gripper_positions = get_gripper_positions_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        assert isinstance(gripper_positions, tuple)
        assert len(gripper_positions) == 2
        np.testing.assert_array_equal(gripper_positions[0], 6.0)
        np.testing.assert_array_equal(gripper_positions[1], 7.0)


def test_get_gripper_velocities_from_lite6_state() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_gripper_velocities_from_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        gripper_velocities = get_gripper_velocities_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        assert isinstance(gripper_velocities, tuple)
        assert len(gripper_velocities) == 2
        np.testing.assert_array_equal(gripper_velocities[0], 14.0)
        np.testing.assert_array_equal(gripper_velocities[1], 15.0)


def test_get_gripper_status_from_lite6_state() -> None:
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_gripper_status_from_lite6_state_vector(
                lite6_model_type=lite6_model_type,
                state_vector=np.zeros(12),
            )

    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        state_vector = np.arange(0, 16, dtype=np.float64)

        lite6_gripper_status = get_gripper_status_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
        )
        assert lite6_gripper_status == Lite6GripperStatus.NEUTRAL

        estimated_lite6_gripper_status = Lite6GripperStatus.OPEN
        estimated_state_vector = add_gripper_status_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            lite6_gripper_status=estimated_lite6_gripper_status,
        )
        lite6_gripper_status = get_gripper_status_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=estimated_state_vector,
        )
        assert lite6_gripper_status == estimated_lite6_gripper_status

        estimated_lite6_gripper_status = Lite6GripperStatus.CLOSED
        estimated_state_vector = add_gripper_status_to_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=state_vector,
            lite6_gripper_status=estimated_lite6_gripper_status,
        )
        lite6_gripper_status = get_gripper_status_from_lite6_state_vector(
            lite6_model_type=lite6_model_type,
            state_vector=estimated_state_vector,
        )
        assert lite6_gripper_status == estimated_lite6_gripper_status


def test_get_default_lite6_joint_positions() -> None:
    # Test for non supported model types.
    for lite6_model_type in (
        Lite6ModelType.NP_GRIPPER,
        Lite6ModelType.RP_GRIPPER,
        Lite6ModelType.V_GRIPPER,
    ):
        with pytest.raises(AssertionError):
            get_default_lite6_joint_positions_vector(
                lite6_model_type=lite6_model_type,
            )

    # For non actuated gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_V_GRIPPER,
        Lite6ModelType.ROBOT_WITHOUT_GRIPPER,
        Lite6ModelType.ROBOT_WITH_UNP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_URP_GRIPPER,
    ):
        positions_vector = get_default_lite6_joint_positions_vector(
            lite6_model_type=lite6_model_type,
        )

        np.testing.assert_array_equal(
            positions_vector,
            np.zeros(6),
        )

    # For parallel gripper robots.
    for lite6_model_type in (
        Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
        Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
    ):
        positions_vector = get_default_lite6_joint_positions_vector(
            lite6_model_type=lite6_model_type,
        )

        np.testing.assert_array_equal(
            positions_vector,
            np.zeros(8),
        )


if __name__ == "__main__":
    execute_pytest_file()
