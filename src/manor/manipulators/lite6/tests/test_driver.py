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
from manor.manipulators.lite6.driver import Lite6Driver, Lite6DriverConfig
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant

# Sentinel speed / accel limits for the driver-construction fixtures. Required fields on
# Lite6DriverConfig (no defaults), so tests have to declare them; the values aren't asserted on,
# the SDK call is mocked.
_TEST_JOINT_SPEED_LIMIT_RAD_S = 1.0
_TEST_JOINT_ACC_LIMIT_RAD_S2 = 2.0


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
        yield Lite6Driver(
            model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL),
            config=Lite6DriverConfig(
                joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S,
                joint_acc_limit_rad_s2=_TEST_JOINT_ACC_LIMIT_RAD_S2,
            ),
        )


@pytest.fixture
def vacuum_driver(arm_mock: mock.MagicMock):
    with mock.patch.object(driver_module, "XArmAPI", return_value=arm_mock):
        yield Lite6Driver(
            model=Lite6Model(variant=Lite6Variant.VACUUM_GRIPPER),
            config=Lite6DriverConfig(
                joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S,
                joint_acc_limit_rad_s2=_TEST_JOINT_ACC_LIMIT_RAD_S2,
            ),
        )


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
        # set_state(READY), then the move-to-PRIME (set_servo_angle). prime() always leaves the arm
        # in mode 0 (POSITION); the operating mode for streaming commands is selected lazily by the
        # write paths via _ensure_mode, so set_mode is called exactly once here (no streaming-mode
        # flip during prime any more).
        arm_mock.clean_warn.assert_called_once()
        arm_mock.clean_error.assert_called_once()
        arm_mock.motion_enable.assert_called_once_with(enable=True)
        arm_mock.set_servo_angle.assert_called_once()
        modes_called = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert modes_called == [0]
        # _current_mode cache is seeded so write paths know they don't need to switch on the first
        # POSITION-shaped call (defensive: we never write angles via mode 0 in production, but the
        # cache is an honest reflection of what xarm_helpers.prime leaves on the arm).
        assert parallel_driver._current_mode == 0

    def test_prime_raises_on_persistent_servo_error(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # If error_code stays non-zero after both bring-up and soft recovery, prime should raise.
        arm_mock.error_code = 23
        with pytest.raises(Lite6DriverError):
            parallel_driver.prime()


class TestHaltResume:
    def test_halt_calls_set_state_stop(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        parallel_driver.halt()
        arm_mock.set_state.assert_called_once_with(state=4)
        # halt must NOT touch motion_enable or disconnect; motors stay energized at current pose.
        arm_mock.motion_enable.assert_not_called()
        arm_mock.disconnect.assert_not_called()
        arm_mock.set_servo_angle.assert_not_called()
        arm_mock.set_mode.assert_not_called()

    def test_resume_calls_set_state_ready(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        parallel_driver.resume()
        arm_mock.set_state.assert_called_once_with(state=0)
        # resume is the symmetric counterpart to halt -- no motion, no mode change.
        arm_mock.set_servo_angle.assert_not_called()
        arm_mock.set_mode.assert_not_called()


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
        # Writes require prime() first so the sticky-mode cache is seeded; otherwise _ensure_mode
        # raises a Lite6DriverError ("write call issued before prime()") -- which is itself covered
        # in TestStickyMode.
        parallel_driver.prime()
        positions = np.linspace(0.0, 0.5, LITE6_ARM_DOF)
        parallel_driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        )
        arm_mock.set_servo_angle_j.assert_called_once()
        kwargs = arm_mock.set_servo_angle_j.call_args.kwargs
        assert kwargs["is_radian"] is True
        assert np.allclose(kwargs["angles"], positions)
        assert kwargs["speed"] == _TEST_JOINT_SPEED_LIMIT_RAD_S
        assert kwargs["mvacc"] == _TEST_JOINT_ACC_LIMIT_RAD_S2

    def test_write_joint_velocities_calls_vc_set_joint_velocity(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        parallel_driver.prime()
        velocities = np.full(LITE6_ARM_DOF, 0.1, dtype=np.float64)
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        arm_mock.vc_set_joint_velocity.assert_called_once()
        kwargs = arm_mock.vc_set_joint_velocity.call_args.kwargs
        assert kwargs["is_radian"] is True
        assert kwargs["duration"] == 0
        assert np.allclose(kwargs["speeds"], velocities)


class TestStickyMode:
    """
    Sticky-mode caching in Lite6Driver._ensure_mode. Verifies switch_mode is invoked only on
    transitions, never on cache hits, and that prime / unprime always leave the arm in POSITION
    (mode 0) so the cache and the firmware agree.
    """

    def test_write_before_prime_raises(self, parallel_driver: Lite6Driver) -> None:
        # _current_mode is None until prime(); writing before prime would silently send commands
        # to an arm in an unknown mode. Raise instead.
        positions = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        with pytest.raises(Lite6DriverError, match="before prime"):
            parallel_driver.write_joint_positions(
                JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
            )

    def test_first_position_write_after_prime_switches_to_servo_position(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # prime() ends in POSITION (mode 0); the first set_servo_angle_j requires SERVO_POSITION
        # (mode 1), so _ensure_mode must call switch_mode (which set_mode's the new mode).
        parallel_driver.prime()
        arm_mock.set_mode.reset_mock()
        positions = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        parallel_driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        )
        modes = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert modes == [1]
        assert parallel_driver._current_mode == 1

    def test_first_velocity_write_after_prime_switches_to_velocity(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        parallel_driver.prime()
        arm_mock.set_mode.reset_mock()
        velocities = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        modes = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert modes == [4]
        assert parallel_driver._current_mode == 4

    def test_repeated_same_shape_writes_skip_switch_mode(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # The whole point of the cache: a streaming policy that publishes only joint velocities
        # pays the switch cost once, then never again.
        parallel_driver.prime()
        velocities = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        for _ in range(5):
            parallel_driver.write_joint_velocities(
                JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
            )
        # set_mode is called exactly twice: once inside prime() for mode 0, once for the first
        # transition into mode 4. The remaining four velocity writes hit the cache.
        modes = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert modes == [0, 4]
        assert arm_mock.vc_set_joint_velocity.call_count == 5

    def test_alternating_shapes_switch_each_transition(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # Worst-case mixed-shape policy: every command flips shape. switch_mode fires on every
        # transition. Confirms _ensure_mode is called per-write (no missed transitions) and that
        # the cache reflects the latest write's mode.
        parallel_driver.prime()
        arm_mock.set_mode.reset_mock()
        positions = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        velocities = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        parallel_driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        )
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        parallel_driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        )
        modes = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert modes == [1, 4, 1]
        assert parallel_driver._current_mode == 1

    def test_unprime_resets_cache_to_position(self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock) -> None:
        # After write_joint_velocities flips us to VELOCITY, unprime takes us back through
        # POSITION (the move-to-ZERO needs mode 0). The cache must follow so a subsequent prime+
        # write doesn't think it's still in VELOCITY and skip a needed switch.
        parallel_driver.prime()
        velocities = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        assert parallel_driver._current_mode == 4
        parallel_driver.unprime()
        assert parallel_driver._current_mode == 0


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
