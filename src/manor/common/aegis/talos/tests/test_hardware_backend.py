"""
Tests for the stale-command watchdog in HardwareManipulatorBackend.

A FakeDriver stands in for the real Lite6Driver so the tests can drive the backend through arbitrary
action-staleness sequences without touching xarm. time.monotonic_ns inside the backend module is
monkey-patched per-test so the watchdog's "now" is fully under test control.
"""

from __future__ import annotations

import attr
import numpy as np
import pytest

from manor.common.aegis.talos import hardware_backend as hardware_backend_module
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend, HardwareManipulatorBackendConfig
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.driver import Lite6DriverConfig
from manor.manipulators.manipulator_driver import IManipulatorDriver

_NUM_DOF = 6
_NUM_EE_DOFS = 2

# Sentinel speed limit for backend-construction fixtures. The watchdog tests don't exercise the
# driver write paths, but Lite6DriverConfig is a required field on HardwareManipulatorBackendConfig.
_TEST_JOINT_SPEED_LIMIT_RAD_S = 1.0


@attr.define
class _FakeDriver(IManipulatorDriver):
    """
    Minimal IManipulatorDriver that records every prime / unprime / halt / resume / write call.
    Read methods are stubbed because the watchdog tests only exercise the write path.
    """

    primed: bool = False
    unprime_count: int = 0
    halt_count: int = 0
    resume_count: int = 0
    write_calls: list[str] = attr.field(factory=list)

    def get_num_dof(self) -> int:
        return _NUM_DOF

    def get_num_ee_dofs(self) -> int:
        return _NUM_EE_DOFS

    def prime(self) -> None:
        self.primed = True

    def unprime(self) -> None:
        self.primed = False
        self.unprime_count += 1

    def halt(self) -> None:
        self.halt_count += 1

    def resume(self) -> None:
        self.resume_count += 1

    def read_joint_positions(self) -> JointPositions:
        return JointPositions(header=TimestampHeader.construct_default(), positions=np.zeros(_NUM_DOF))

    def read_joint_velocities(self) -> JointVelocities:
        return JointVelocities(header=TimestampHeader.construct_default(), velocities=np.zeros(_NUM_DOF))

    def read_ee_positions(self) -> EEPositions | None:
        return None

    def read_ee_velocities(self) -> EEVelocities | None:
        return None

    def write_joint_positions(self, joint_positions: JointPositions) -> None:
        self.write_calls.append("joint_positions")

    def write_joint_velocities(self, joint_velocities: JointVelocities) -> None:
        self.write_calls.append("joint_velocities")

    def write_ee_positions(self, ee_positions: EEPositions) -> None:
        self.write_calls.append("ee_positions")

    def write_ee_velocities(self, ee_velocities: EEVelocities) -> None:
        self.write_calls.append("ee_velocities")


def _make_command() -> JointEECommand:
    header = TimestampHeader(monotonic_ns=1, system_ns=2)
    return JointEECommand(
        header=header,
        joint_command=JointCommand(
            header=header,
            joint_positions=JointPositions(header=header, positions=np.zeros(_NUM_DOF)),
        ),
    )


@pytest.fixture
def fake_driver() -> _FakeDriver:
    return _FakeDriver()


@pytest.fixture
def backend(fake_driver: _FakeDriver) -> HardwareManipulatorBackend:
    # 1 / 0.3s ~= 3.33 Hz -- watchdog trips after ~300 ms of silence.
    return HardwareManipulatorBackend(
        driver=fake_driver,
        config=HardwareManipulatorBackendConfig(
            minimum_watchdog_frequency_hz=1.0 / 0.3,
            lite6_driver_config=Lite6DriverConfig(joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S),
        ),
    )


@pytest.fixture
def fake_now(monkeypatch: pytest.MonkeyPatch):
    """
    Replace time.monotonic_ns inside hardware_backend with a controllable counter so the watchdog
    sees the system time the test wants it to see.
    """
    state = {"now_ns": 1_000_000_000}

    def _now() -> int:
        return state["now_ns"]

    monkeypatch.setattr(hardware_backend_module.time, "monotonic_ns", _now)
    return state


class TestStartStopLifecycle:
    def test_start_primes_driver_and_clears_stopped(
        self, backend: HardwareManipulatorBackend, fake_driver: _FakeDriver
    ) -> None:
        backend.start()
        assert fake_driver.primed is True
        assert backend._stopped is False
        assert backend._latest_action_monotonic_ns == 0

    def test_stop_unprimes_driver(self, backend: HardwareManipulatorBackend, fake_driver: _FakeDriver) -> None:
        backend.start()
        backend.stop()
        assert fake_driver.unprime_count == 1


class TestWatchdogStartupGrace:
    def test_send_without_any_action_does_not_halt(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        # No action has ever been observed -- staleness check is gated on
        # _latest_action_monotonic_ns > 0, so the backend stays armed (does not halt).
        fake_now["now_ns"] += int(10.0 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._stopped is False
        assert fake_driver.halt_count == 0

    def test_send_without_any_action_drops_command(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
    ) -> None:
        # Before any action arrives the JointEECommand on the wire is the default-constructed one,
        # which carries an empty joint_positions array that crashes the xarm SDK. Backend must drop
        # those sends.
        backend.start()
        backend.send_joint_ee_command(_make_command())
        assert fake_driver.write_calls == []


class TestWatchdogTrip:
    def test_fresh_action_keeps_backend_armed(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        # Time advances within the threshold.
        fake_now["now_ns"] += int(0.1 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._stopped is False
        assert fake_driver.halt_count == 0
        assert fake_driver.write_calls == ["joint_positions"]

    def test_stale_action_halts_arm_without_unpriming(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        # Stamp an action header at "now"; then advance time past the threshold so the next send
        # detects staleness.
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())
        # Watchdog tripped: backend halted, command dropped, but the arm was NOT unprimed -- the
        # arm holds its current pose with motors energized.
        assert backend._stopped is True
        assert fake_driver.halt_count == 1
        assert fake_driver.unprime_count == 0
        assert fake_driver.write_calls == []

    def test_subsequent_sends_while_stopped_are_dropped_without_re_halt(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())  # trip
        assert fake_driver.halt_count == 1

        # Repeated sends while stopped: short-circuit at the _stopped check, no second halt.
        backend.send_joint_ee_command(_make_command())
        backend.send_joint_ee_command(_make_command())
        assert fake_driver.halt_count == 1
        assert fake_driver.write_calls == []


class TestWatchdogAutoResume:
    def test_fresh_action_after_halt_auto_resumes(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        # Trip the watchdog.
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._stopped is True
        assert fake_driver.resume_count == 0

        # Metis restarts and publishes a fresh action with a strictly-newer header.
        fake_now["now_ns"] += int(1.0 * 1e9)
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        # Auto-resume: backend cleared _stopped and called driver.resume().
        assert backend._stopped is False
        assert fake_driver.resume_count == 1

        # Subsequent send goes through to the driver from the current pose.
        backend.send_joint_ee_command(_make_command())
        assert fake_driver.write_calls == ["joint_positions"]

    def test_pet_with_same_header_does_not_resume(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        # Trip the watchdog.
        last_header = TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0)
        backend.pet_watchdog(last_header)
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._stopped is True

        # The LCM subscriber keeps serving the same pre-kill header -- pet_watchdog must not
        # treat that as fresh and must not auto-resume.
        backend.pet_watchdog(last_header)
        assert backend._stopped is True
        assert fake_driver.resume_count == 0

    def test_restart_clears_stopped_state(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._stopped is True

        backend.start()  # operator re-launches kylos
        assert backend._stopped is False
        assert backend._latest_action_monotonic_ns == 0
        assert fake_driver.primed is True


class TestPetWatchdogAdvancement:
    def test_pet_does_not_advance_on_repeated_header(
        self,
        backend: HardwareManipulatorBackend,
        fake_now: dict,
    ) -> None:
        backend.start()
        # Same header notified repeatedly (LCM subscriber holds the last message). Internal stamp must
        # not advance with later system time, so once the threshold elapses the watchdog will still
        # trip on the next send.
        repeating_header = TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0)
        backend.pet_watchdog(repeating_header)
        first_stamp = backend._latest_action_monotonic_ns
        fake_now["now_ns"] += int(1.0 * 1e9)
        backend.pet_watchdog(repeating_header)
        assert backend._latest_action_monotonic_ns == first_stamp


class TestHardwareManipulatorBackendConfig:
    def test_minimum_watchdog_frequency_hz_is_required(self) -> None:
        # No default -- every aegis YAML must set it explicitly.
        with pytest.raises(TypeError):
            HardwareManipulatorBackendConfig()

    def test_driver_config_blocks_are_optional(self) -> None:
        # The per-driver config blocks default to None at the parse layer; aegis.py requires the block
        # matching the configured manipulator type only when it actually builds the hardware driver, so a
        # YAML declares just the block for the arm it runs.
        config = HardwareManipulatorBackendConfig(minimum_watchdog_frequency_hz=3.0)
        assert config.lite6_driver_config is None
        assert config.rebot_b601_dm_driver_config is None

    def test_rejects_non_positive_minimum_watchdog_frequency_hz(self) -> None:
        with pytest.raises(ValueError):
            HardwareManipulatorBackendConfig(
                minimum_watchdog_frequency_hz=0.0,
                lite6_driver_config=Lite6DriverConfig(joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S),
            )

    def test_from_yaml_dict(self) -> None:
        config = HardwareManipulatorBackendConfig.from_yaml_dict(
            {
                "minimum_watchdog_frequency_hz": 2.0,
                "lite6_driver_config": {"joint_speed_limit_rad_s": 0.5},
            }
        )
        assert config.minimum_watchdog_frequency_hz == 2.0
        assert config.lite6_driver_config.joint_speed_limit_rad_s == 0.5

    def test_from_yaml_dict_rebot_b601_dm(self) -> None:
        config = HardwareManipulatorBackendConfig.from_yaml_dict(
            {
                "minimum_watchdog_frequency_hz": 2.0,
                "rebot_b601_dm_driver_config": {
                    "joint_speed_limit_rad_s": 0.5,
                    "max_command_error_rad": 0.15,
                    "gripper_torque_ratio": 0.07,
                },
            }
        )
        assert config.minimum_watchdog_frequency_hz == 2.0
        assert config.lite6_driver_config is None
        assert config.rebot_b601_dm_driver_config.joint_speed_limit_rad_s == 0.5
        assert config.rebot_b601_dm_driver_config.max_command_error_rad == 0.15
        assert config.rebot_b601_dm_driver_config.gripper_torque_ratio == 0.07

    def test_from_yaml_dict_missing_field_raises(self) -> None:
        from manor.common.exceptions import AegisConfigError

        with pytest.raises(AegisConfigError):
            HardwareManipulatorBackendConfig.from_yaml_dict({})


if __name__ == "__main__":
    run_manor_tests()
