"""
Tests for RebotB601DmDriver.

motorbridge is a hard dependency, but the tests inject MagicMock controller / motor objects in place of the
real serial bus so the bring-up / read / write paths can be exercised against the SDK contract without
hardware. Construction is patched at the Controller import-site inside motorbridge_helpers so the bus's lazy
connect() lands on the mock.

time.sleep is patched out because the shared helpers include settle pauses at every enable / mode step.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest
from motorbridge import Mode

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import RebotB601DmDriverError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.rebot_b601_dm import motorbridge_helpers as motorbridge_helpers_module
from manor.manipulators.rebot_b601_dm.driver import (
    RebotB601DmArmControlMode,
    RebotB601DmDriver,
    RebotB601DmDriverConfig,
)
from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.model import REBOT_B601_DM_ARM_DOF, RebotB601DmModel
from manor.manipulators.rebot_b601_dm.motorbridge_helpers import (
    REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M,
    REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD,
    REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX,
    REBOT_B601_DM_MAX_COMMAND_ERROR_CEILING_RAD,
    REBOT_B601_DM_MOTOR_SPECS,
    RebotB601DmBus,
)
from manor.manipulators.rebot_b601_dm.variant import RebotB601DmVariant

_TEST_JOINT_SPEED_LIMIT_RAD_S = 1.0
_TEST_MAX_COMMAND_ERROR_RAD = 0.2
_TEST_GRIPPER_TORQUE_RATIO = 0.07


class FakeBusHardware:
    """
    Mock-side state for the fake motor bus: per-motor MagicMocks whose get_state returns a mutable
    (pos, vel, torq) snapshot, plus the controller MagicMock they hang off. Position-command sends
    (send_pos_vel / send_mit) teleport the fake motor to the commanded position so convergence polls exit
    on their first read; set teleport_on_send False to emulate a blocked / stalled joint.
    """

    def __init__(self) -> None:
        self.controller = mock.MagicMock()
        self.motors: dict[str, mock.MagicMock] = {}
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.teleport_on_send = True

        id_to_name = {spec.send_id: spec.name for spec in REBOT_B601_DM_MOTOR_SPECS}

        def _add_motor(send_id: int, feedback_id: int, model: str) -> mock.MagicMock:
            name = id_to_name[send_id]
            motor = mock.MagicMock()
            self.positions.setdefault(name, 0.0)
            self.velocities.setdefault(name, 0.0)

            def _get_state(_name: str = name) -> SimpleNamespace:
                return SimpleNamespace(
                    pos=self.positions[_name],
                    vel=self.velocities[_name],
                    torq=0.0,
                    status_code=0,
                )

            motor.get_state.side_effect = _get_state

            def _send_pos_vel(pos: float, vlim: float, _name: str = name) -> None:
                if self.teleport_on_send:
                    self.positions[_name] = pos

            def _send_mit(pos: float, vel: float, kp: float, kd: float, tau: float, _name: str = name) -> None:
                # A kp = 0 command exerts no positional pull; only teleport for position-tracking sends.
                if self.teleport_on_send and kp > 0.0:
                    self.positions[_name] = pos

            motor.send_pos_vel.side_effect = _send_pos_vel
            motor.send_mit.side_effect = _send_mit
            self.motors[name] = motor
            return motor

        self.controller.add_damiao_motor.side_effect = _add_motor

    def set_arm_positions(self, positions: np.ndarray) -> None:
        for i in range(REBOT_B601_DM_ARM_DOF):
            self.positions[REBOT_B601_DM_MOTOR_SPECS[i].name] = float(positions[i])

    def arm_motor(self, index: int) -> mock.MagicMock:
        return self.motors[REBOT_B601_DM_MOTOR_SPECS[index].name]

    @property
    def gripper_motor(self) -> mock.MagicMock:
        return self.motors["gripper"]


@pytest.fixture
def hw() -> FakeBusHardware:
    return FakeBusHardware()


@pytest.fixture(autouse=True)
def _no_sleep():
    with mock.patch("time.sleep"):
        yield


def _make_driver(hw: FakeBusHardware, arm_control_mode: RebotB601DmArmControlMode):
    controller_cls = mock.MagicMock()
    controller_cls.from_dm_serial.return_value = hw.controller
    patcher = mock.patch.object(motorbridge_helpers_module, "Controller", controller_cls)
    patcher.start()
    driver = RebotB601DmDriver(
        model=RebotB601DmModel(variant=RebotB601DmVariant.PARALLEL_GRIPPER),
        config=RebotB601DmDriverConfig(
            joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S,
            max_command_error_rad=_TEST_MAX_COMMAND_ERROR_RAD,
            gripper_torque_ratio=_TEST_GRIPPER_TORQUE_RATIO,
            arm_control_mode=arm_control_mode,
        ),
    )
    return driver, patcher


@pytest.fixture
def driver(hw: FakeBusHardware):
    driver, patcher = _make_driver(hw, RebotB601DmArmControlMode.MIT)
    yield driver
    patcher.stop()


@pytest.fixture
def pos_vel_driver(hw: FakeBusHardware):
    driver, patcher = _make_driver(hw, RebotB601DmArmControlMode.POS_VEL)
    yield driver
    patcher.stop()


def _header(monotonic_ns: int) -> TimestampHeader:
    return TimestampHeader(monotonic_ns=monotonic_ns, system_ns=monotonic_ns)


class TestConfigValidation:
    def test_torque_ratio_above_ceiling_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(
                joint_speed_limit_rad_s=1.0,
                max_command_error_rad=0.15,
                gripper_torque_ratio=REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX + 0.01,
            )

    def test_torque_ratio_zero_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(joint_speed_limit_rad_s=1.0, max_command_error_rad=0.15, gripper_torque_ratio=0.0)

    def test_speed_limit_zero_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(joint_speed_limit_rad_s=0.0, max_command_error_rad=0.15, gripper_torque_ratio=0.07)

    def test_command_error_above_ceiling_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(
                joint_speed_limit_rad_s=1.0,
                max_command_error_rad=REBOT_B601_DM_MAX_COMMAND_ERROR_CEILING_RAD + 0.01,
                gripper_torque_ratio=0.07,
            )

    def test_arm_control_mode_defaults_to_mit(self) -> None:
        config = RebotB601DmDriverConfig(
            joint_speed_limit_rad_s=1.0, max_command_error_rad=0.15, gripper_torque_ratio=0.07
        )
        assert config.arm_control_mode is RebotB601DmArmControlMode.MIT


class TestConstruction:
    def test_construction_does_not_touch_hardware(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        # Constructing the driver must not open the serial bridge; only prime() connects.
        hw.controller.add_damiao_motor.assert_not_called()

    def test_dof_accessors(self, driver: RebotB601DmDriver) -> None:
        assert driver.get_num_dof() == REBOT_B601_DM_ARM_DOF
        assert driver.get_num_ee_dofs() == 1


class TestPrime:
    def test_prime_sequence(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        # All seven motors registered, bus enabled.
        assert hw.controller.add_damiao_motor.call_count == len(REBOT_B601_DM_MOTOR_SPECS)
        hw.controller.enable_all.assert_called()
        # Arm joints in MIT (the torque-bounded default); gripper in FORCE_POS.
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.arm_motor(i).ensure_mode.assert_any_call(Mode.MIT, mock.ANY)
        hw.gripper_motor.ensure_mode.assert_any_call(Mode.FORCE_POS, mock.ANY)
        # Gripper closed softly: FORCE_POS to motor zero at the configured torque ratio.
        pos, _vlim, ratio = hw.gripper_motor.send_force_pos.call_args[0]
        assert pos == 0.0
        assert ratio == _TEST_GRIPPER_TORQUE_RATIO
        # Arm ended at PRIME.
        prime_target = RebotB601DmJointConfiguration.PRIME.get_joint_positions_vector()
        positions, _ = driver._bus.read_arm_state()
        np.testing.assert_allclose(positions, prime_target)

    def test_prime_is_repeatable(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        driver.prime()
        # The bus reconnect is idempotent: motors registered exactly once.
        assert hw.controller.add_damiao_motor.call_count == len(REBOT_B601_DM_MOTOR_SPECS)

    def test_prime_pos_vel_mode_writes_loop_gains(self, pos_vel_driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        # In pos_vel mode prime ends by switching the arm into POS_VEL, which writes the four cascade
        # gain registers per joint.
        pos_vel_driver.prime()
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.arm_motor(i).ensure_mode.assert_any_call(Mode.POS_VEL, mock.ANY)
            assert hw.arm_motor(i).write_register_f32.call_count == 4


class TestUnprime:
    def test_unprime_parks_and_disables(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        driver.unprime()
        rest_target = RebotB601DmJointConfiguration.REST.get_joint_positions_vector()
        positions, _ = driver._bus.read_arm_state()
        np.testing.assert_allclose(positions, rest_target)
        hw.controller.disable_all.assert_called()

    def test_write_after_unprime_raises(self, driver: RebotB601DmDriver) -> None:
        driver.prime()
        driver.unprime()
        with pytest.raises(RebotB601DmDriverError):
            driver.write_joint_positions(JointPositions(header=_header(1), positions=np.zeros(REBOT_B601_DM_ARM_DOF)))


class TestWriteJointPositions:
    def test_write_before_prime_raises(self, driver: RebotB601DmDriver) -> None:
        with pytest.raises(RebotB601DmDriverError):
            driver.write_joint_positions(JointPositions(header=_header(1), positions=np.zeros(REBOT_B601_DM_ARM_DOF)))

    def test_first_write_seeds_from_measured(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        measured = np.array([0.1, -0.2, -0.3, 0.05, -0.1, 0.2])
        hw.set_arm_positions(measured)
        # First write after prime: the limiter has no history, so the commanded value is the measured
        # pose regardless of the requested target.
        target = np.full(REBOT_B601_DM_ARM_DOF, 2.0)
        driver.write_joint_positions(JointPositions(header=_header(1), positions=target))
        for i in range(REBOT_B601_DM_ARM_DOF):
            commanded = hw.arm_motor(i).send_mit.call_args[0][0]
            assert commanded == pytest.approx(measured[i])

    def test_rate_limited_advance(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        hw.set_arm_positions(np.zeros(REBOT_B601_DM_ARM_DOF))
        headers = iter([_header(0), _header(int(0.1e9))])
        with mock.patch.object(driver, "_now_timestamp_header", side_effect=lambda: next(headers)):
            target = np.full(REBOT_B601_DM_ARM_DOF, 1.0)
            driver.write_joint_positions(JointPositions(header=_header(0), positions=target))
            driver.write_joint_positions(JointPositions(header=_header(1), positions=target))
        # Second write advances by at most joint_speed_limit_rad_s * dt = 1.0 * 0.1 = 0.1 rad.
        for i in range(REBOT_B601_DM_ARM_DOF):
            commanded = hw.arm_motor(i).send_mit.call_args[0][0]
            assert commanded == pytest.approx(0.1)

    def test_command_error_clamped_against_measured_when_blocked(
        self, driver: RebotB601DmDriver, hw: FakeBusHardware
    ) -> None:
        # THE arm torque bound: with a blocked joint (measured position stalled) the commanded position
        # may never lead the measured one by more than max_command_error_rad, no matter how far the rate
        # limiter has advanced. Torque is then bounded at ~kp * clamp per joint.
        driver.prime()
        hw.set_arm_positions(np.zeros(REBOT_B601_DM_ARM_DOF))
        hw.teleport_on_send = False
        target = np.full(REBOT_B601_DM_ARM_DOF, 2.0)
        headers = iter([_header(int(i * 1e9)) for i in range(10)])
        with mock.patch.object(driver, "_now_timestamp_header", side_effect=lambda: next(headers)):
            for i in range(1, 6):
                driver.write_joint_positions(JointPositions(header=_header(i), positions=target))
        # After 5 writes the rate limiter alone would have advanced 4+ rad worth of steps, but the
        # measured position never moved -- so the command must sit exactly at the clamp.
        for i in range(REBOT_B601_DM_ARM_DOF):
            commanded = hw.arm_motor(i).send_mit.call_args[0][0]
            assert commanded == pytest.approx(_TEST_MAX_COMMAND_ERROR_RAD)

    def test_pos_vel_mode_uses_pos_vel_sends(self, pos_vel_driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        pos_vel_driver.prime()
        measured = np.zeros(REBOT_B601_DM_ARM_DOF)
        hw.set_arm_positions(measured)
        pos_vel_driver.write_joint_positions(
            JointPositions(header=_header(1), positions=np.full(REBOT_B601_DM_ARM_DOF, 1.0))
        )
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.arm_motor(i).send_pos_vel.assert_called()


class TestWriteJointVelocities:
    def test_mit_mode_streams_damping_command(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        # In the MIT default, velocity commands stream as pure damping (kp = 0): torque =
        # kd * velocity_error, bounded by construction; no mode switch happens.
        driver.prime()
        velocities = np.array([0.1, -0.1, 0.2, -0.2, 0.3, -0.3])
        driver.write_joint_velocities(JointVelocities(header=_header(1), velocities=velocities))
        for i in range(REBOT_B601_DM_ARM_DOF):
            pos, vel, kp, _kd, _tau = hw.arm_motor(i).send_mit.call_args[0]
            assert vel == pytest.approx(velocities[i])
            assert kp == 0.0
            vel_mode_calls = [c for c in hw.arm_motor(i).ensure_mode.call_args_list if c[0][0] == Mode.VEL]
            assert not vel_mode_calls

    def test_pos_vel_mode_switches_to_vel_and_sends(
        self, pos_vel_driver: RebotB601DmDriver, hw: FakeBusHardware
    ) -> None:
        pos_vel_driver.prime()
        velocities = np.array([0.1, -0.1, 0.2, -0.2, 0.3, -0.3])
        pos_vel_driver.write_joint_velocities(JointVelocities(header=_header(1), velocities=velocities))
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.arm_motor(i).ensure_mode.assert_any_call(Mode.VEL, mock.ANY)
            assert hw.arm_motor(i).send_vel.call_args[0][0] == pytest.approx(velocities[i])

    def test_pos_vel_mode_switch_is_sticky(self, pos_vel_driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        pos_vel_driver.prime()
        velocities = JointVelocities(header=_header(1), velocities=np.zeros(REBOT_B601_DM_ARM_DOF))
        pos_vel_driver.write_joint_velocities(velocities)
        vel_mode_calls = [c for c in hw.arm_motor(0).ensure_mode.call_args_list if c[0][0] == Mode.VEL]
        pos_vel_driver.write_joint_velocities(velocities)
        vel_mode_calls_after = [c for c in hw.arm_motor(0).ensure_mode.call_args_list if c[0][0] == Mode.VEL]
        assert len(vel_mode_calls_after) == len(vel_mode_calls)


class TestGripper:
    def test_write_ee_positions_maps_width_to_motor(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        hw.gripper_motor.send_force_pos.reset_mock()
        # Half open maps to half the motor open position.
        half_open_width = REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M / 2.0
        driver.write_ee_positions(EEPositions(header=_header(1), positions=np.array([half_open_width])))
        pos, _vlim, ratio = hw.gripper_motor.send_force_pos.call_args[0]
        assert pos == pytest.approx(REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD / 2.0)
        assert ratio == _TEST_GRIPPER_TORQUE_RATIO

    def test_gripper_commands_always_torque_capped(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        # Every gripper send during and after prime must carry the configured (safe) torque ratio;
        # nothing may command full torque.
        driver.prime()
        driver.write_ee_positions(EEPositions(header=_header(1), positions=np.array([0.0])))
        driver.write_ee_velocities(EEVelocities(header=_header(2), velocities=np.array([0.05])))
        for call in hw.gripper_motor.send_force_pos.call_args_list:
            assert call[0][2] <= REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX

    def test_bus_send_reclamps_excessive_ratio(self, hw: FakeBusHardware) -> None:
        # Defence in depth: even if a caller bypasses the config validator, the bus clamps the ratio at
        # the module ceiling before it reaches the motor.
        controller_cls = mock.MagicMock()
        controller_cls.from_dm_serial.return_value = hw.controller
        with mock.patch.object(motorbridge_helpers_module, "Controller", controller_cls):
            bus = RebotB601DmBus()
            bus.connect()
            bus.send_gripper_force_pos(motor_rad=0.0, torque_ratio=0.9)
        _pos, _vlim, ratio = hw.gripper_motor.send_force_pos.call_args[0]
        assert ratio == pytest.approx(REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX)

    def test_write_ee_velocities_direction_semantics(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        hw.gripper_motor.send_force_pos.reset_mock()
        # Positive width rate drives toward fully open.
        driver.write_ee_velocities(EEVelocities(header=_header(1), velocities=np.array([0.05])))
        assert hw.gripper_motor.send_force_pos.call_args[0][0] == pytest.approx(REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD)
        # Negative drives toward closed.
        driver.write_ee_velocities(EEVelocities(header=_header(2), velocities=np.array([-0.05])))
        assert hw.gripper_motor.send_force_pos.call_args[0][0] == pytest.approx(0.0)
        # Zero holds the current opening: the half-open FEEDBACK position round-trips through the width
        # mapping into the half-open COMMAND position (the frames differ; echoing feedback back as a
        # command would drive the gripper closed).
        hw.positions["gripper"] = REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD / 2.0
        driver.write_ee_velocities(EEVelocities(header=_header(3), velocities=np.array([0.0])))
        assert hw.gripper_motor.send_force_pos.call_args[0][0] == pytest.approx(
            REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD / 2.0, rel=1e-3
        )

    def test_read_ee_positions_maps_motor_to_width(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        hw.positions["gripper"] = REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD / 2.0
        ee_positions = driver.read_ee_positions()
        assert ee_positions is not None
        np.testing.assert_allclose(ee_positions.positions, [REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M / 2.0])


class TestReadJointState:
    def test_read_positions_and_velocities(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        measured = np.array([0.1, -0.2, -0.3, 0.05, -0.1, 0.2])
        hw.set_arm_positions(measured)
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.velocities[REBOT_B601_DM_MOTOR_SPECS[i].name] = 0.01 * (i + 1)
        np.testing.assert_allclose(driver.read_joint_positions().positions, measured)
        np.testing.assert_allclose(
            driver.read_joint_velocities().velocities,
            [0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        )


class TestHaltResume:
    def test_halt_latches_current_pose_and_refuses_writes(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        held = np.array([0.1, -0.4, -0.2, 0.0, 0.1, -0.1])
        hw.set_arm_positions(held)
        driver.halt()
        # The measured pose was latched as the MIT hold target.
        for i in range(REBOT_B601_DM_ARM_DOF):
            assert hw.arm_motor(i).send_mit.call_args[0][0] == pytest.approx(held[i])
        with pytest.raises(RebotB601DmDriverError):
            driver.write_joint_positions(JointPositions(header=_header(1), positions=held))

    def test_resume_allows_writes_again(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        driver.halt()
        driver.resume()
        driver.write_joint_positions(JointPositions(header=_header(1), positions=np.zeros(REBOT_B601_DM_ARM_DOF)))

    def test_halt_in_vel_mode_zeroes_velocity(self, pos_vel_driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        pos_vel_driver.prime()
        pos_vel_driver.write_joint_velocities(
            JointVelocities(header=_header(1), velocities=np.full(REBOT_B601_DM_ARM_DOF, 0.5))
        )
        pos_vel_driver.halt()
        for i in range(REBOT_B601_DM_ARM_DOF):
            assert hw.arm_motor(i).send_vel.call_args[0][0] == pytest.approx(0.0)

    def test_resume_is_idempotent(self, driver: RebotB601DmDriver) -> None:
        driver.prime()
        driver.resume()
        driver.resume()


if __name__ == "__main__":
    run_manor_tests()
