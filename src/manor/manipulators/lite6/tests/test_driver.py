"""
Tests for ``Lite6Driver``.

xarm-python-sdk is a hard dependency, but the tests inject a
``MagicMock`` arm in place of the real ``XArmAPI`` so the read/write
paths can be exercised against the SDK contract without needing a
physical robot on the network.
"""

from __future__ import annotations

from unittest import mock

import numpy as np
import pytest

from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import Lite6DriverError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6 import driver as driver_module
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_arm_mock(positions: np.ndarray | None = None, velocities: np.ndarray | None = None) -> mock.MagicMock:
    """
    Build a MagicMock that behaves like xarm's ``XArmAPI`` for the
    methods Lite6Driver actually calls.
    """
    arm = mock.MagicMock()
    # All control-plane calls (clean_error / motion_enable / set_mode /
    # set_state / set_servo_angle_j / vc_set_joint_velocity / gripper)
    # return a plain int status code; default success.
    arm.clean_error.return_value = 0
    arm.motion_enable.return_value = 0
    arm.set_mode.return_value = 0
    arm.set_state.return_value = 0
    arm.set_servo_angle_j.return_value = 0
    arm.vc_set_joint_velocity.return_value = 0
    arm.open_lite6_gripper.return_value = 0
    arm.close_lite6_gripper.return_value = 0
    arm.stop_lite6_gripper.return_value = 0
    arm.set_vacuum_gripper.return_value = 0
    arm.emergency_stop.return_value = 0

    # The xarm SDK pads to 7 elements regardless of arm DOF -- mimic that.
    seven_zeros = [0.0] * 7
    pos_list = list(positions) + [0.0] * (7 - len(positions)) if positions is not None else seven_zeros
    vel_list = list(velocities) + [0.0] * (7 - len(velocities)) if velocities is not None else seven_zeros
    arm.get_joint_states.return_value = (0, (pos_list, vel_list, seven_zeros))
    return arm


@pytest.fixture
def parallel_driver() -> Lite6Driver:
    return Lite6Driver(model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL))


@pytest.fixture
def vacuum_driver() -> Lite6Driver:
    return Lite6Driver(model=Lite6Model(variant=Lite6Variant.VACUUM_GRIPPER))


def _prime_with_mock(driver: Lite6Driver, arm: mock.MagicMock) -> None:
    """
    Bypass the real XArmAPI constructor by patching the import-level
    symbol, then call prime so the driver wires up its internal state.
    """
    with mock.patch.object(driver_module, "XArmAPI", return_value=arm):
        driver.prime()


class TestDofAccessors:
    def test_get_num_dof_delegates_to_model(self, parallel_driver: Lite6Driver) -> None:
        assert parallel_driver.get_num_dof() == parallel_driver.model.get_num_dof()

    def test_get_num_eef_dofs_delegates_to_model(
        self,
        parallel_driver: Lite6Driver,
        vacuum_driver: Lite6Driver,
    ) -> None:
        assert parallel_driver.get_num_eef_dofs() == parallel_driver.model.get_num_eef_dofs()
        assert vacuum_driver.get_num_eef_dofs() == vacuum_driver.model.get_num_eef_dofs()


class TestPrimeUnprime:
    def test_prime_invokes_expected_setup_calls(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)

        arm.clean_error.assert_called_once()
        arm.motion_enable.assert_called_once_with(enable=True)
        arm.set_mode.assert_called_once()
        arm.set_state.assert_called_once()

    def test_unprime_without_prior_prime_is_noop(self, parallel_driver: Lite6Driver) -> None:
        # Should not raise even though no SDK handle was ever wired in.
        parallel_driver.unprime()

    def test_unprime_releases_arm_handle(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)
        parallel_driver.unprime()
        # After unprime, calls on the driver should fail again because
        # the SDK handle is gone.
        with pytest.raises(Lite6DriverError):
            parallel_driver.read_joint_positions()


class TestReadCalls:
    def test_read_joint_positions_slices_to_arm_dof(self, parallel_driver: Lite6Driver) -> None:
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        arm = _make_arm_mock(positions=positions)
        _prime_with_mock(parallel_driver, arm)

        result = parallel_driver.read_joint_positions()
        assert isinstance(result, JointPositions)
        assert result.positions.shape == (LITE6_ARM_DOF,)
        np.testing.assert_array_equal(result.positions, positions)
        # Header should be a freshly stamped TimestampHeader.
        assert isinstance(result.header, TimestampHeader)
        assert result.header.monotonic_ns > 0

    def test_read_joint_velocities_slices_to_arm_dof(self, parallel_driver: Lite6Driver) -> None:
        velocities = np.array([0.01, -0.02, 0.03, -0.04, 0.05, -0.06], dtype=np.float64)
        arm = _make_arm_mock(velocities=velocities)
        _prime_with_mock(parallel_driver, arm)

        result = parallel_driver.read_joint_velocities()
        assert isinstance(result, JointVelocities)
        np.testing.assert_array_equal(result.velocities, velocities)

    def test_read_eef_state_returns_none(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)
        assert parallel_driver.read_eef_positions() is None
        assert parallel_driver.read_eef_velocities() is None

    def test_read_without_prime_raises(self, parallel_driver: Lite6Driver) -> None:
        with pytest.raises(Lite6DriverError):
            parallel_driver.read_joint_positions()

    def test_read_propagates_sdk_failure(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)
        arm.get_joint_states.return_value = (1, ([0.0] * 7, [0.0] * 7, [0.0] * 7))
        with pytest.raises(Lite6DriverError):
            parallel_driver.read_joint_positions()


class TestWriteJointCalls:
    def test_write_joint_positions_calls_set_servo_angle_j(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)

        positions = np.linspace(0.0, 0.5, LITE6_ARM_DOF)
        parallel_driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        )
        arm.set_servo_angle_j.assert_called_once()
        kwargs = arm.set_servo_angle_j.call_args.kwargs
        assert kwargs["is_radian"] is True
        assert np.allclose(kwargs["angles"], positions)

    def test_write_joint_velocities_calls_vc_set_joint_velocity(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)

        velocities = np.full(LITE6_ARM_DOF, 0.1, dtype=np.float64)
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        arm.vc_set_joint_velocity.assert_called_once()
        kwargs = arm.vc_set_joint_velocity.call_args.kwargs
        assert kwargs["is_radian"] is True
        assert kwargs["duration"] == 0
        assert np.allclose(kwargs["speeds"], velocities)


class TestWriteEEFCallsParallelGripper:
    def test_open_position_threshold_dispatches_open(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)

        # Above-threshold position vector -> open
        parallel_driver.write_eef_positions(
            EEFPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.array([0.008, -0.008], dtype=np.float64),
            )
        )
        arm.open_lite6_gripper.assert_called_once()
        arm.close_lite6_gripper.assert_not_called()

    def test_zero_position_dispatches_close(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)

        parallel_driver.write_eef_positions(
            EEFPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.zeros(2, dtype=np.float64),
            )
        )
        arm.close_lite6_gripper.assert_called_once()
        arm.open_lite6_gripper.assert_not_called()

    def test_zero_velocity_dispatches_stop(self, parallel_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(parallel_driver, arm)

        parallel_driver.write_eef_velocities(
            EEFVelocities(
                header=TimestampHeader.from_system_time(),
                velocities=np.zeros(2, dtype=np.float64),
            )
        )
        arm.stop_lite6_gripper.assert_called_once()


class TestWriteEEFCallsVacuumGripper:
    def test_open_position_threshold_turns_vacuum_on(self, vacuum_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(vacuum_driver, arm)

        vacuum_driver.write_eef_positions(
            EEFPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.array([0.01], dtype=np.float64),
            )
        )
        arm.set_vacuum_gripper.assert_called_once_with(on=True)

    def test_zero_position_turns_vacuum_off(self, vacuum_driver: Lite6Driver) -> None:
        arm = _make_arm_mock()
        _prime_with_mock(vacuum_driver, arm)

        vacuum_driver.write_eef_positions(
            EEFPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.zeros(1, dtype=np.float64),
            )
        )
        arm.set_vacuum_gripper.assert_called_once_with(on=False)


if __name__ == "__main__":
    run_manor_tests()
