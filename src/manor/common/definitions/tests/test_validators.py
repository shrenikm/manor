"""
Validator failure tests across the primitive definitions. The happy paths
are already exercised by each definition's round-trip tests; this file
exercises the rejection branch for each shape / dtype / value rule.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.cartesian_pose_trajectory import CartesianPoseTrajectory
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.cartesian_twist_trajectory import CartesianTwistTrajectory
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_positions_trajectory import EEPositionsTrajectory
from manor.common.definitions.ee_state_trajectory import EEStateTrajectory
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.ee_velocities_trajectory import EEVelocitiesTrajectory
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_positions_trajectory import JointPositionsTrajectory
from manor.common.definitions.joint_state_trajectory import JointStateTrajectory
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.joint_velocities_trajectory import JointVelocitiesTrajectory
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.enums import DepthEncoding, ImageEncoding
from manor.common.exceptions import InvalidDefinitionError
from manor.common.testing_utils import run_manor_tests


def _h() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=0, system_ns=0)


class TestTimestampHeader:
    def test_rejects_negative_ns(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            TimestampHeader(monotonic_ns=-1, system_ns=0)
        with pytest.raises(InvalidDefinitionError):
            TimestampHeader(monotonic_ns=0, system_ns=-1)


class TestJointPositions:
    def test_rejects_2d_positions(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointPositions(header=_h(), positions=np.zeros((2, 3), dtype=np.float64))

    def test_rejects_wrong_dtype(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointPositions(header=_h(), positions=np.zeros(3, dtype=np.int32))


class TestJointVelocities:
    def test_rejects_2d_velocities(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointVelocities(header=_h(), velocities=np.zeros((2, 3), dtype=np.float64))


class TestEEPositions:
    def test_rejects_2d(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            EEPositions(header=_h(), positions=np.zeros((2, 1), dtype=np.float64))


class TestEEVelocities:
    def test_rejects_2d(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            EEVelocities(header=_h(), velocities=np.zeros((2, 1), dtype=np.float64))


class TestCartesianPose:
    def test_rejects_translation_wrong_shape(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianPose(
                header=_h(),
                translation=np.zeros(2, dtype=np.float64),
                orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            )

    def test_rejects_non_unit_quaternion(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianPose(
                header=_h(),
                translation=np.zeros(3, dtype=np.float64),
                orientation=np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64),
            )

    def test_rejects_quaternion_wrong_length(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianPose(
                header=_h(),
                translation=np.zeros(3, dtype=np.float64),
                orientation=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            )


class TestCartesianTwist:
    def test_rejects_wrong_shape(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianTwist(
                header=_h(),
                linear=np.zeros(2, dtype=np.float64),
                angular=np.zeros(3, dtype=np.float64),
            )


class TestJointPositionsTrajectory:
    def test_rejects_empty_trajectory(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointPositionsTrajectory(
                header=_h(),
                times=np.zeros(0, dtype=np.float64),
                joint_positions_array=np.zeros((0, 3), dtype=np.float64),
            )

    def test_rejects_unsorted_times(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointPositionsTrajectory(
                header=_h(),
                times=np.array([1.0, 0.5], dtype=np.float64),
                joint_positions_array=np.zeros((2, 3), dtype=np.float64),
            )

    def test_rejects_length_mismatch(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointPositionsTrajectory(
                header=_h(),
                times=np.array([0.0, 1.0], dtype=np.float64),
                joint_positions_array=np.zeros((3, 3), dtype=np.float64),
            )

    def test_rejects_1d_payload(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointPositionsTrajectory(
                header=_h(),
                times=np.array([0.0], dtype=np.float64),
                joint_positions_array=np.zeros(3, dtype=np.float64),
            )


class TestJointVelocitiesTrajectory:
    def test_rejects_empty(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointVelocitiesTrajectory(
                header=_h(),
                times=np.zeros(0, dtype=np.float64),
                joint_velocities_array=np.zeros((0, 3), dtype=np.float64),
            )


class TestJointStateTrajectory:
    def test_rejects_mismatched_dof_columns(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            JointStateTrajectory(
                header=_h(),
                times=np.array([0.0, 1.0], dtype=np.float64),
                joint_positions_array=np.zeros((2, 3), dtype=np.float64),
                joint_velocities_array=np.zeros((2, 4), dtype=np.float64),
            )


class TestCartesianPoseTrajectory:
    def test_rejects_translations_wrong_cols(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianPoseTrajectory(
                header=_h(),
                times=np.array([0.0], dtype=np.float64),
                translations_array=np.zeros((1, 2), dtype=np.float64),
                orientations_array=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            )

    def test_rejects_orientations_wrong_cols(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianPoseTrajectory(
                header=_h(),
                times=np.array([0.0], dtype=np.float64),
                translations_array=np.zeros((1, 3), dtype=np.float64),
                orientations_array=np.zeros((1, 5), dtype=np.float64),
            )


class TestCartesianTwistTrajectory:
    def test_rejects_empty(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            CartesianTwistTrajectory(
                header=_h(),
                times=np.zeros(0, dtype=np.float64),
                linear_array=np.zeros((0, 3), dtype=np.float64),
                angular_array=np.zeros((0, 3), dtype=np.float64),
            )


class TestEEPositionsTrajectory:
    def test_rejects_empty(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            EEPositionsTrajectory(
                header=_h(),
                times=np.zeros(0, dtype=np.float64),
                ee_positions_array=np.zeros((0, 1), dtype=np.float64),
            )


class TestEEVelocitiesTrajectory:
    def test_rejects_empty(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            EEVelocitiesTrajectory(
                header=_h(),
                times=np.zeros(0, dtype=np.float64),
                ee_velocities_array=np.zeros((0, 1), dtype=np.float64),
            )


class TestEEStateTrajectory:
    def test_rejects_mismatched_columns(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            EEStateTrajectory(
                header=_h(),
                times=np.array([0.0], dtype=np.float64),
                ee_positions_array=np.zeros((1, 2), dtype=np.float64),
                ee_velocities_array=np.zeros((1, 3), dtype=np.float64),
            )


class TestRGBImageData:
    def test_rejects_negative_dims(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            RGBImageData(
                header=_h(),
                height=-1,
                width=10,
                encoding=ImageEncoding.RAW_RGB8,
                data=b"",
            )


class TestDepthImageData:
    def test_rejects_negative_depth_scale(self) -> None:
        with pytest.raises(InvalidDefinitionError):
            DepthImageData(
                header=_h(),
                height=1,
                width=1,
                encoding=DepthEncoding.RAW_FLOAT32_M,
                data=b"\x00\x00\x00\x00",
                depth_scale=-1.0,
            )


if __name__ == "__main__":
    run_manor_tests()
