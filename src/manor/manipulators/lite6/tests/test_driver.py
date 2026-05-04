"""
Tests for ``Lite6Driver``.

xarm-python-sdk is a hard dependency, but the tests inject a
``MagicMock`` arm in place of the real ``XArmAPI`` so the read/write
paths can be exercised against the SDK contract without needing a
physical robot on the network. Construction is patched at the
``XArmAPI`` import-site (the helpers module also imports it) so the
driver's eager arm-construction in ``__init__`` lands on the mock.

``time.sleep`` is patched out for the prime / unprime paths because the
shared helpers include a 2 s motion_enable settle that adds dead time
to every test that exercises bring-up.
"""

from __future__ import annotations

from unittest import mock

import numpy as np
import pytest

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import Lite6DriverError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6 import driver as driver_module
from manor.manipulators.lite6 import xarm_helpers as xarm_helpers_module
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_arm_mock(positions: np.ndarray | None = None, velocities: np.ndarray | None = None) -> mock.MagicMock:
    """
    Build a MagicMock that behaves like xarm's XArmAPI for the methods Lite6Driver and its shared
    helpers actually call. All return-code calls default to success (0); error_code / warn_code default
    to 0 so the helpers' clean-state assertion passes; mode is a property the helpers poll after a
    set_mode call to confirm the heartbeat catches up.
    """
    arm = mock.MagicMock()
    arm.clean_warn.return_value = 0
    arm.clean_error.return_value = 0
    arm.motion_enable.return_value = 0
    arm.set_mode.return_value = 0
    arm.set_state.return_value = 0
    arm.set_servo_angle.return_value = 0
    arm.set_servo_angle_j.return_value = 0
    arm.vc_set_joint_velocity.return_value = 0
    arm.open_lite6_gripper.return_value = 0
    arm.close_lite6_gripper.return_value = 0
    arm.stop_lite6_gripper.return_value = 0
    arm.set_vacuum_gripper.return_value = 0

    arm.error_code = 0
    arm.warn_code = 0
    # The helpers poll arm.mode after every switch_mode; mirror the firmware behaviour by having the
    # mock's mode property update whenever set_mode(mode=X) is called, so the poll loop in switch_mode
    # exits immediately rather than spinning to its deadline.
    arm.mode = 0

    def _set_mode(*, mode: int) -> int:
        arm.mode = int(mode)
        return 0

    arm.set_mode.side_effect = _set_mode

    seven_zeros = [0.0] * 7
    pos_list = list(positions) + [0.0] * (7 - len(positions)) if positions is not None else seven_zeros
    vel_list = list(velocities) + [0.0] * (7 - len(velocities)) if velocities is not None else seven_zeros
    arm.get_joint_states.return_value = (0, (pos_list, vel_list, seven_zeros))
    return arm


@pytest.fixture
def arm_mock() -> mock.MagicMock:
    return _make_arm_mock()


@pytest.fixture
def parallel_driver(arm_mock: mock.MagicMock):
    """
    Patch the driver-module's XArmAPI symbol so eager arm-construction in Lite6Driver.__init__ lands
    on the mock instead of trying to reach the real robot.
    """
    with mock.patch.object(driver_module, "XArmAPI", return_value=arm_mock):
        yield Lite6Driver(model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL))


@pytest.fixture
def vacuum_driver(arm_mock: mock.MagicMock):
    with mock.patch.object(driver_module, "XArmAPI", return_value=arm_mock):
        yield Lite6Driver(model=Lite6Model(variant=Lite6Variant.VACUUM_GRIPPER))


@pytest.fixture(autouse=True)
def _no_sleep():
    """
    The shared helpers sleep ~2 s in motion_enable_settle and ~1 s during soft recovery. Skip that
    wait in unit tests; we only care about call ordering.
    """
    with mock.patch.object(xarm_helpers_module, "time") as fake_time:
        fake_time.sleep.return_value = None
        # switch_mode reads time.monotonic() once to set a deadline, then again each poll iteration.
        # The arm-mock's set_mode side-effect updates arm.mode so the poll exits on the first read,
        # but if any test ever races the mode update we still want time to advance monotonically -- a
        # counter keeps the poll loop bounded as a safety net.
        counter = {"now": 0.0}

        def _monotonic() -> float:
            counter["now"] += 1.0
            return counter["now"]

        fake_time.monotonic.side_effect = _monotonic
        yield


class TestDofAccessors:
    def test_get_num_dof_delegates_to_model(self, parallel_driver: Lite6Driver) -> None:
        assert parallel_driver.get_num_dof() == parallel_driver.model.get_num_dof()

    def test_get_num_ee_dofs_delegates_to_model(
        self,
        parallel_driver: Lite6Driver,
        vacuum_driver: Lite6Driver,
    ) -> None:
        assert parallel_driver.get_num_ee_dofs() == parallel_driver.model.get_num_ee_dofs()
        assert vacuum_driver.get_num_ee_dofs() == vacuum_driver.model.get_num_ee_dofs()


class TestPrime:
    def test_prime_runs_full_bring_up_sequence(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        parallel_driver.prime()

        # The shared helpers ordering: clean_warn, clean_error, motion_enable(True), set_mode(0),
        # set_state(0), then the move-to-PRIME (set_servo_angle), then a switch into the streaming
        # mode (STOP -> set_mode(1) -> READY).
        arm_mock.clean_warn.assert_called_once()
        arm_mock.clean_error.assert_called_once()
        arm_mock.motion_enable.assert_called_once_with(enable=True)
        arm_mock.set_servo_angle.assert_called_once()
        # set_mode is called twice: once during connect (mode 0) and once during the streaming-mode
        # switch (mode 1).
        modes_called = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert 0 in modes_called and 1 in modes_called

    def test_prime_raises_on_persistent_servo_error(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # If error_code stays non-zero after both bring-up and soft recovery, prime should raise.
        arm_mock.error_code = 23
        with pytest.raises(Lite6DriverError):
            parallel_driver.prime()


class TestUnprime:
    def test_unprime_switches_to_position_moves_to_zero_and_stops(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # The arm-mock's set_mode side-effect mirrors firmware: arm.mode flips to whatever was last
        # requested. unprime requests mode 0 (POSITION) before moving to ZERO; we just verify the
        # call sequence on the mock.
        parallel_driver.unprime()

        # Mode 0 should have been requested (the switch_mode call inside unprime).
        modes_requested = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert 0 in modes_requested
        # set_servo_angle is the move-to-ZERO trajectory.
        arm_mock.set_servo_angle.assert_called_once()
        # The final SDK call is set_state(STOP) (state=4).
        last_state_call = arm_mock.set_state.call_args_list[-1]
        assert last_state_call.kwargs["state"] == 4
        # Critical: unprime must NOT disconnect / disable motors (the watchdog re-primes on this
        # same handle).
        arm_mock.motion_enable.assert_not_called()
        arm_mock.disconnect.assert_not_called()


class TestReadCalls:
    def test_read_joint_positions_slices_to_arm_dof(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        arm_mock.get_joint_states.return_value = (
            0,
            (list(positions) + [0.0], [0.0] * 7, [0.0] * 7),
        )
        result = parallel_driver.read_joint_positions()
        assert isinstance(result, JointPositions)
        assert result.positions.shape == (LITE6_ARM_DOF,)
        np.testing.assert_array_equal(result.positions, positions)
        assert isinstance(result.header, TimestampHeader)
        assert result.header.monotonic_ns > 0

    def test_read_joint_velocities_slices_to_arm_dof(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        velocities = np.array([0.01, -0.02, 0.03, -0.04, 0.05, -0.06], dtype=np.float64)
        arm_mock.get_joint_states.return_value = (
            0,
            ([0.0] * 7, list(velocities) + [0.0], [0.0] * 7),
        )
        result = parallel_driver.read_joint_velocities()
        assert isinstance(result, JointVelocities)
        np.testing.assert_array_equal(result.velocities, velocities)

    def test_read_ee_state_returns_none(self, parallel_driver: Lite6Driver) -> None:
        assert parallel_driver.read_ee_positions() is None
        assert parallel_driver.read_ee_velocities() is None

    def test_read_propagates_sdk_failure(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        arm_mock.get_joint_states.return_value = (1, ([0.0] * 7, [0.0] * 7, [0.0] * 7))
        with pytest.raises(Lite6DriverError):
            parallel_driver.read_joint_positions()


class TestWriteJointCalls:
    def test_write_joint_positions_calls_set_servo_angle_j(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        positions = np.linspace(0.0, 0.5, LITE6_ARM_DOF)
        parallel_driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        )
        arm_mock.set_servo_angle_j.assert_called_once()
        kwargs = arm_mock.set_servo_angle_j.call_args.kwargs
        assert kwargs["is_radian"] is True
        assert np.allclose(kwargs["angles"], positions)

    def test_write_joint_velocities_calls_vc_set_joint_velocity(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        velocities = np.full(LITE6_ARM_DOF, 0.1, dtype=np.float64)
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        arm_mock.vc_set_joint_velocity.assert_called_once()
        kwargs = arm_mock.vc_set_joint_velocity.call_args.kwargs
        assert kwargs["is_radian"] is True
        assert kwargs["duration"] == 0
        assert np.allclose(kwargs["speeds"], velocities)


class TestWriteEECallsParallelGripper:
    def test_open_position_threshold_dispatches_open(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        parallel_driver.write_ee_positions(
            EEPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.array([0.008, -0.008], dtype=np.float64),
            )
        )
        arm_mock.open_lite6_gripper.assert_called_once()
        arm_mock.close_lite6_gripper.assert_not_called()

    def test_zero_position_dispatches_close(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        parallel_driver.write_ee_positions(
            EEPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.zeros(2, dtype=np.float64),
            )
        )
        arm_mock.close_lite6_gripper.assert_called_once()
        arm_mock.open_lite6_gripper.assert_not_called()

    def test_zero_velocity_dispatches_stop(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        parallel_driver.write_ee_velocities(
            EEVelocities(
                header=TimestampHeader.from_system_time(),
                velocities=np.zeros(2, dtype=np.float64),
            )
        )
        arm_mock.stop_lite6_gripper.assert_called_once()


class TestWriteEECallsVacuumGripper:
    def test_open_position_threshold_turns_vacuum_on(
        self, vacuum_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        vacuum_driver.write_ee_positions(
            EEPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.array([0.01], dtype=np.float64),
            )
        )
        arm_mock.set_vacuum_gripper.assert_called_once_with(on=True)

    def test_zero_position_turns_vacuum_off(self, vacuum_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        vacuum_driver.write_ee_positions(
            EEPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.zeros(1, dtype=np.float64),
            )
        )
        arm_mock.set_vacuum_gripper.assert_called_once_with(on=False)


if __name__ == "__main__":
    run_manor_tests()
