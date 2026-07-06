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
from manor.manipulators.rebot_b601_dm import motorbridge_helpers as helpers_module
from manor.manipulators.rebot_b601_dm.driver import RebotB601DmDriver, RebotB601DmDriverConfig
from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.model import REBOT_B601_DM_ARM_DOF, RebotB601DmModel
from manor.manipulators.rebot_b601_dm.motorbridge_helpers import (
    REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD,
    REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX,
    REBOT_B601_DM_MOTOR_SPECS,
    RebotB601DmBus,
)
from manor.manipulators.rebot_b601_dm.variant import RebotB601DmVariant

_TEST_JOINT_SPEED_LIMIT_RAD_S = 1.0
_TEST_GRIPPER_TORQUE_RATIO = 0.07


class FakeBusHardware:
    """
    Mock-side state for the fake motor bus: per-motor MagicMocks whose get_state returns a mutable
    (pos, vel, torq) snapshot, plus the controller MagicMock they hang off.
    """

    def __init__(self) -> None:
        self.controller = mock.MagicMock()
        self.motors: dict[str, mock.MagicMock] = {}
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}

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

            # POS_VEL targets teleport the fake motor to the commanded position so convergence polls
            # (move_to_configuration during prime / unprime) exit on their first read.
            def _send_pos_vel(pos: float, vlim: float, _name: str = name) -> None:
                self.positions[_name] = pos

            motor.send_pos_vel.side_effect = _send_pos_vel
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


@pytest.fixture
def driver(hw: FakeBusHardware):
    controller_cls = mock.MagicMock()
    controller_cls.from_dm_serial.return_value = hw.controller
    with mock.patch.object(helpers_module, "Controller", controller_cls):
        yield RebotB601DmDriver(
            model=RebotB601DmModel(variant=RebotB601DmVariant.PARALLEL_GRIPPER),
            config=RebotB601DmDriverConfig(
                joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S,
                gripper_torque_ratio=_TEST_GRIPPER_TORQUE_RATIO,
            ),
        )


def _header(monotonic_ns: int) -> TimestampHeader:
    return TimestampHeader(monotonic_ns=monotonic_ns, system_ns=monotonic_ns)


class TestConfigValidation:
    def test_torque_ratio_above_ceiling_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(
                joint_speed_limit_rad_s=1.0,
                gripper_torque_ratio=REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX + 0.01,
            )

    def test_torque_ratio_zero_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(joint_speed_limit_rad_s=1.0, gripper_torque_ratio=0.0)

    def test_speed_limit_zero_rejected(self) -> None:
        with pytest.raises(ValueError):
            RebotB601DmDriverConfig(joint_speed_limit_rad_s=0.0, gripper_torque_ratio=0.07)


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
        # Arm joints in POS_VEL with the loop-gain registers written; gripper in FORCE_POS.
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.arm_motor(i).ensure_mode.assert_any_call(Mode.POS_VEL, mock.ANY)
            assert hw.arm_motor(i).write_register_f32.call_count == 4
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
            commanded = hw.arm_motor(i).send_pos_vel.call_args[0][0]
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
            commanded = hw.arm_motor(i).send_pos_vel.call_args[0][0]
            assert commanded == pytest.approx(0.1)


class TestWriteJointVelocities:
    def test_switches_to_vel_mode_and_sends(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        velocities = np.array([0.1, -0.1, 0.2, -0.2, 0.3, -0.3])
        driver.write_joint_velocities(JointVelocities(header=_header(1), velocities=velocities))
        for i in range(REBOT_B601_DM_ARM_DOF):
            hw.arm_motor(i).ensure_mode.assert_any_call(Mode.VEL, mock.ANY)
            assert hw.arm_motor(i).send_vel.call_args[0][0] == pytest.approx(velocities[i])

    def test_mode_switch_is_sticky(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        velocities = JointVelocities(header=_header(1), velocities=np.zeros(REBOT_B601_DM_ARM_DOF))
        driver.write_joint_velocities(velocities)
        vel_mode_calls = [c for c in hw.arm_motor(0).ensure_mode.call_args_list if c[0][0] == Mode.VEL]
        driver.write_joint_velocities(velocities)
        vel_mode_calls_after = [c for c in hw.arm_motor(0).ensure_mode.call_args_list if c[0][0] == Mode.VEL]
        assert len(vel_mode_calls_after) == len(vel_mode_calls)


class TestGripper:
    def test_write_ee_positions_maps_width_to_motor(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        hw.gripper_motor.send_force_pos.reset_mock()
        # Half open: width 0.0715 -> motor -2.5 rad.
        driver.write_ee_positions(EEPositions(header=_header(1), positions=np.array([0.0715])))
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
        with mock.patch.object(helpers_module, "Controller", controller_cls):
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
        # Zero holds the current motor position.
        hw.positions["gripper"] = -1.25
        driver.write_ee_velocities(EEVelocities(header=_header(3), velocities=np.array([0.0])))
        assert hw.gripper_motor.send_force_pos.call_args[0][0] == pytest.approx(-1.25)

    def test_read_ee_positions_maps_motor_to_width(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        hw.positions["gripper"] = REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD / 2.0
        ee_positions = driver.read_ee_positions()
        assert ee_positions is not None
        np.testing.assert_allclose(ee_positions.positions, [0.0715])


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
        # The measured pose was latched as the POS_VEL hold target.
        for i in range(REBOT_B601_DM_ARM_DOF):
            assert hw.arm_motor(i).send_pos_vel.call_args[0][0] == pytest.approx(held[i])
        with pytest.raises(RebotB601DmDriverError):
            driver.write_joint_positions(JointPositions(header=_header(1), positions=held))

    def test_resume_allows_writes_again(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        driver.halt()
        driver.resume()
        driver.write_joint_positions(JointPositions(header=_header(1), positions=np.zeros(REBOT_B601_DM_ARM_DOF)))

    def test_halt_in_vel_mode_zeroes_velocity(self, driver: RebotB601DmDriver, hw: FakeBusHardware) -> None:
        driver.prime()
        driver.write_joint_velocities(
            JointVelocities(header=_header(1), velocities=np.full(REBOT_B601_DM_ARM_DOF, 0.5))
        )
        driver.halt()
        for i in range(REBOT_B601_DM_ARM_DOF):
            assert hw.arm_motor(i).send_vel.call_args[0][0] == pytest.approx(0.0)

    def test_resume_is_idempotent(self, driver: RebotB601DmDriver) -> None:
        driver.prime()
        driver.resume()
        driver.resume()


if __name__ == "__main__":
    run_manor_tests()
