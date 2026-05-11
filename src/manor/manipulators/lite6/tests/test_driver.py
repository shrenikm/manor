"""
Tests for Lite6Driver.

xarm-python-sdk is a hard dependency, but the tests inject a MagicMock arm in place of the real XArmAPI so
the read/write paths can be exercised against the SDK contract without needing a physical robot on the
network. Construction is patched at the XArmAPI import-site (the helpers module also imports it) so the
driver's eager arm-construction in __init__ lands on the mock.

time.sleep is patched out for the prime / unprime paths because the shared helpers include a 2 s
motion_enable settle that adds dead time to every test that exercises bring-up.
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

# Sentinel speed limit for the driver-construction fixtures. Required field on Lite6DriverConfig (no
# default), so tests have to declare one. Most tests don't assert on the value (the SDK call is mocked); the
# rate-limiter tests in TestRateLimiter pick their own.
_TEST_JOINT_SPEED_LIMIT_RAD_S = 1.0


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
            config=Lite6DriverConfig(joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S),
        )


@pytest.fixture
def vacuum_driver(arm_mock: mock.MagicMock):
    with mock.patch.object(driver_module, "XArmAPI", return_value=arm_mock):
        yield Lite6Driver(
            model=Lite6Model(variant=Lite6Variant.VACUUM_GRIPPER),
            config=Lite6DriverConfig(joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S),
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

    def test_resume_before_prime_calls_set_state_ready(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # Pre-prime _current_mode is None, so there's no streaming controller to re-arm. resume
        # falls back to a bare set_state(READY).
        parallel_driver.resume()
        arm_mock.set_state.assert_called_once_with(state=0)
        arm_mock.set_servo_angle.assert_not_called()
        arm_mock.set_mode.assert_not_called()

    def test_resume_after_prime_re_arms_via_switch_mode(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # After prime _current_mode is POSITION. set_state(READY) alone leaves the firmware's
        # streaming context dormant after a halt's STOP, so resume re-runs the full switch_mode
        # dance (set_state STOP -> set_mode -> set_state READY).
        parallel_driver.prime()
        arm_mock.set_mode.reset_mock()
        arm_mock.set_state.reset_mock()
        parallel_driver.resume()
        modes = [c.kwargs["mode"] for c in arm_mock.set_mode.call_args_list]
        assert modes == [0]  # POSITION re-flipped to itself
        states = [c.kwargs["state"] for c in arm_mock.set_state.call_args_list]
        # switch_mode emits set_state(STOP=4) then set_state(READY=0).
        assert states == [4, 0]

    def test_resume_in_velocity_mode_re_arms_velocity(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # The motivating bug: a velocity-streaming policy halted by the watchdog, then resumed,
        # would have its commands silently ignored by the firmware. Resume must re-flip mode 4
        # specifically (not just set_state READY) so streaming reactivates.
        parallel_driver.prime()
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=np.zeros(LITE6_ARM_DOF))
        )
        assert parallel_driver._current_mode == 4  # VELOCITY
        arm_mock.set_mode.reset_mock()
        arm_mock.set_state.reset_mock()
        parallel_driver.resume()
        modes = [c.kwargs["mode"] for c in arm_mock.set_mode.call_args_list]
        assert modes == [4]
        states = [c.kwargs["state"] for c in arm_mock.set_state.call_args_list]
        assert states == [4, 0]


class TestUnprime:
    def test_unprime_switches_to_position_moves_to_rest_and_stops(
        self, parallel_driver: Lite6Driver, arm_mock: mock.MagicMock
    ) -> None:
        # The arm-mock's set_mode side-effect mirrors firmware: arm.mode flips to whatever was last
        # requested. unprime requests mode 0 (POSITION) before moving to REST; we just verify the
        # call sequence on the mock.
        parallel_driver.unprime()

        # Mode 0 should have been requested (the switch_mode call inside unprime).
        modes_requested = [call.kwargs["mode"] for call in arm_mock.set_mode.call_args_list]
        assert 0 in modes_requested
        # set_servo_angle is the move-to-REST trajectory.
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
        # Per-call speed / mvacc are NOT passed -- the firmware ignores them for set_servo_angle_j.
        # Speed limiting happens client-side in _compute_rate_limited_target; see TestRateLimiter.
        assert "speed" not in kwargs
        assert "mvacc" not in kwargs
        assert len(kwargs["angles"]) == LITE6_ARM_DOF

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
        # POSITION (the move-to-REST needs mode 0). The cache must follow so a subsequent prime+
        # write doesn't think it's still in VELOCITY and skip a needed switch.
        parallel_driver.prime()
        velocities = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        parallel_driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        )
        assert parallel_driver._current_mode == 4
        parallel_driver.unprime()
        assert parallel_driver._current_mode == 0


class TestRateLimiter:
    """
    Client-side joint-position rate limiter inside Lite6Driver.write_joint_positions. The xarm
    SDK's per-call speed= argument on set_servo_angle_j is reserved/ignored by the firmware for
    mode 1 streaming, so the driver clamps each commanded position to advance by at most
    joint_speed_limit_rad_s * dt from the previously commanded pose.
    """

    @staticmethod
    def _patch_clock(driver: Lite6Driver, monotonic_ns_values: list[int]) -> None:
        """
        Override driver._now_timestamp_header so the rate limiter sees a controlled stream of
        monotonic timestamps. Each value yields a TimestampHeader with that monotonic_ns and a
        sentinel system_ns -- the limiter only reads monotonic_ns. Patching the instance method
        (rather than the global time module or TimestampHeader classmethod) avoids breaking
        other code in the process that constructs headers via from_system_time.
        """
        it = iter(monotonic_ns_values)
        driver._now_timestamp_header = lambda: TimestampHeader(monotonic_ns=next(it), system_ns=0)  # type: ignore[method-assign]

    def _make_driver(self, arm_mock: mock.MagicMock, speed_limit_rad_s: float) -> Lite6Driver:
        with mock.patch.object(driver_module, "XArmAPI", return_value=arm_mock):
            return Lite6Driver(
                model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL),
                config=Lite6DriverConfig(joint_speed_limit_rad_s=speed_limit_rad_s),
            )

    def test_first_write_after_prime_seeds_from_measured_pose(self, arm_mock: mock.MagicMock) -> None:
        # Set the measured pose to something distinguishable so we can tell the SDK call carried
        # the measured pose (limiter seed) and not the policy target.
        measured = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        arm_mock.get_joint_states.return_value = (0, (list(measured) + [0.0], [0.0] * 7, [0.0] * 7))
        driver = self._make_driver(arm_mock, speed_limit_rad_s=1.0)
        self._patch_clock(driver, [1_000_000_000])
        driver.prime()
        target = np.full(LITE6_ARM_DOF, 1.5, dtype=np.float64)
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        kwargs = arm_mock.set_servo_angle_j.call_args.kwargs
        np.testing.assert_allclose(kwargs["angles"], measured)

    def test_second_write_advances_full_step_when_within_speed_budget(self, arm_mock: mock.MagicMock) -> None:
        # First call seeds at zeros (the default measured pose). Second call has dt=1s and a
        # speed limit of 100 rad/s, so max_step=100 rad easily covers a 0.5-rad target delta --
        # the commanded pose lands exactly on the target.
        driver = self._make_driver(arm_mock, speed_limit_rad_s=100.0)
        self._patch_clock(driver, [0, 1_000_000_000])
        driver.prime()
        target = np.full(LITE6_ARM_DOF, 0.5, dtype=np.float64)
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        kwargs = arm_mock.set_servo_angle_j.call_args_list[-1].kwargs
        np.testing.assert_allclose(kwargs["angles"], target)

    def test_second_write_clips_to_max_step_when_target_exceeds_speed_budget(self, arm_mock: mock.MagicMock) -> None:
        # speed=0.1 rad/s and dt=1s gives max_step=0.1 rad. Target at +1.0 rad far exceeds the
        # budget; commanded pose advances by exactly 0.1 rad past the seed (zeros).
        driver = self._make_driver(arm_mock, speed_limit_rad_s=0.1)
        self._patch_clock(driver, [0, 1_000_000_000])
        driver.prime()
        target = np.full(LITE6_ARM_DOF, 1.0, dtype=np.float64)
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        kwargs = arm_mock.set_servo_angle_j.call_args_list[-1].kwargs
        np.testing.assert_allclose(kwargs["angles"], np.full(LITE6_ARM_DOF, 0.1))

    def test_clip_is_per_joint_signed(self, arm_mock: mock.MagicMock) -> None:
        # Mixed positive / negative deltas larger than the budget per joint clip toward the target
        # in each joint independently. speed=0.1 rad/s, dt=1s -> max_step=0.1 rad/joint.
        driver = self._make_driver(arm_mock, speed_limit_rad_s=0.1)
        self._patch_clock(driver, [0, 1_000_000_000])
        driver.prime()
        target = np.array([1.0, -1.0, 0.05, -0.05, 0.0, 1.0], dtype=np.float64)
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        kwargs = arm_mock.set_servo_angle_j.call_args_list[-1].kwargs
        # +1.0 -> +0.1 (clipped), -1.0 -> -0.1 (clipped), +/- 0.05 stay (within budget),
        # 0.0 stays 0, +1.0 -> +0.1 (clipped).
        expected = np.array([0.1, -0.1, 0.05, -0.05, 0.0, 0.1], dtype=np.float64)
        np.testing.assert_allclose(kwargs["angles"], expected)

    def test_resume_resets_limiter(self, arm_mock: mock.MagicMock) -> None:
        # After resume the limiter must re-seed from the measured pose -- the arm may have
        # decelerated under halt and drifted away from where the previous _last_commanded_position
        # was set. We swap the measured pose between the second seed and the resume to prove the
        # post-resume write reads the current measurement, not the cached one.
        first_measured = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        second_measured = np.full(LITE6_ARM_DOF, 0.7, dtype=np.float64)
        arm_mock.get_joint_states.return_value = (
            0,
            (list(first_measured) + [0.0], [0.0] * 7, [0.0] * 7),
        )
        driver = self._make_driver(arm_mock, speed_limit_rad_s=1.0)
        self._patch_clock(driver, [0, 1_000_000_000, 2_000_000_000])
        driver.prime()
        # Seed at zeros.
        driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=np.zeros(LITE6_ARM_DOF))
        )
        # Halt + resume: limiter should drop its cached seed.
        driver.halt()
        arm_mock.get_joint_states.return_value = (
            0,
            (list(second_measured) + [0.0], [0.0] * 7, [0.0] * 7),
        )
        driver.resume()
        target = np.full(LITE6_ARM_DOF, 1.5, dtype=np.float64)
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        kwargs = arm_mock.set_servo_angle_j.call_args_list[-1].kwargs
        # Post-resume write is treated as a fresh seed, so it sends the new measured pose.
        np.testing.assert_allclose(kwargs["angles"], second_measured)

    def test_velocity_to_position_transition_resets_limiter(self, arm_mock: mock.MagicMock) -> None:
        # An intervening VELOCITY phase moves the arm away from _last_commanded_position. The
        # next position write must re-seed from measured rather than continuing from the stale
        # cache, so swapping the mock's measured pose between the two position writes proves the
        # second write reads the new measurement.
        first_measured = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
        second_measured = np.full(LITE6_ARM_DOF, 0.4, dtype=np.float64)
        arm_mock.get_joint_states.return_value = (
            0,
            (list(first_measured) + [0.0], [0.0] * 7, [0.0] * 7),
        )
        driver = self._make_driver(arm_mock, speed_limit_rad_s=1.0)
        self._patch_clock(driver, [0, 1_000_000_000, 2_000_000_000])
        driver.prime()
        driver.write_joint_positions(
            JointPositions(header=TimestampHeader.from_system_time(), positions=np.zeros(LITE6_ARM_DOF))
        )
        driver.write_joint_velocities(
            JointVelocities(header=TimestampHeader.from_system_time(), velocities=np.zeros(LITE6_ARM_DOF))
        )
        arm_mock.get_joint_states.return_value = (
            0,
            (list(second_measured) + [0.0], [0.0] * 7, [0.0] * 7),
        )
        target = np.full(LITE6_ARM_DOF, 1.0, dtype=np.float64)
        driver.write_joint_positions(JointPositions(header=TimestampHeader.from_system_time(), positions=target))
        kwargs = arm_mock.set_servo_angle_j.call_args_list[-1].kwargs
        np.testing.assert_allclose(kwargs["angles"], second_measured)


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
