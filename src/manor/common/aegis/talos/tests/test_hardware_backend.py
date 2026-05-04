"""
Tests for the stale-command watchdog in HardwareManipulatorBackend.

A FakeDriver stands in for the real Lite6Driver so the tests can drive
the backend through arbitrary action-staleness sequences without
touching xarm. ``time.monotonic_ns`` inside the backend module is
monkey-patched per-test so the watchdog's "now" is fully under test
control.
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
from manor.manipulators.manipulator_driver import IManipulatorDriver

_NUM_DOF = 6
_NUM_EE_DOFS = 2


@attr.define
class _FakeDriver(IManipulatorDriver):
    """
    Minimal IManipulatorDriver that records every prime / unprime / write call. Read methods are
    stubbed because the watchdog tests only exercise the write path.
    """

    primed: bool = False
    unprime_count: int = 0
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
    return HardwareManipulatorBackend(
        driver=fake_driver,
        config=HardwareManipulatorBackendConfig(stale_command_threshold_s=0.3),
    )


@pytest.fixture
def fake_now(monkeypatch: pytest.MonkeyPatch):
    """
    Replace time.monotonic_ns inside hardware_backend with a controllable counter so the watchdog
    sees the wall-clock the test wants it to see.
    """
    state = {"now_ns": 1_000_000_000}

    def _now() -> int:
        return state["now_ns"]

    monkeypatch.setattr(hardware_backend_module.time, "monotonic_ns", _now)
    return state


class TestStartStopLifecycle:
    def test_start_primes_driver_and_resets_park(
        self, backend: HardwareManipulatorBackend, fake_driver: _FakeDriver
    ) -> None:
        backend.start()
        assert fake_driver.primed is True
        assert backend._parked is False
        assert backend._latest_action_monotonic_ns == 0

    def test_stop_unprimes_driver(self, backend: HardwareManipulatorBackend, fake_driver: _FakeDriver) -> None:
        backend.start()
        backend.stop()
        assert fake_driver.unprime_count == 1


class TestWatchdogStartupGrace:
    def test_send_without_any_action_does_not_park(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        # Time advances well past the threshold but no action has ever been observed -- staleness
        # check is gated on _latest_action_monotonic_ns > 0, so the backend stays armed and the
        # command is forwarded.
        fake_now["now_ns"] += int(10.0 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._parked is False
        assert fake_driver.write_calls == ["joint_positions"]


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
        assert backend._parked is False
        assert fake_driver.unprime_count == 0
        assert fake_driver.write_calls == ["joint_positions"]

    def test_stale_action_parks_and_drops_command(
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
        # Watchdog tripped: backend parked, driver unprimed, command dropped.
        assert backend._parked is True
        assert fake_driver.unprime_count == 1
        assert fake_driver.write_calls == []

    def test_park_is_sticky_against_fresh_action(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())  # trip
        assert backend._parked is True

        # A fresh action arrives after a Metis restart; the spec says we must NOT auto-rearm.
        fake_now["now_ns"] += int(1.0 * 1e9)
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        backend.send_joint_ee_command(_make_command())
        assert backend._parked is True
        assert fake_driver.write_calls == []
        # No second unprime: the backend short-circuits at the parked check before re-checking
        # staleness.
        assert fake_driver.unprime_count == 1

    def test_restart_clears_park_state(
        self,
        backend: HardwareManipulatorBackend,
        fake_driver: _FakeDriver,
        fake_now: dict,
    ) -> None:
        backend.start()
        backend.pet_watchdog(TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0))
        fake_now["now_ns"] += int(0.5 * 1e9)
        backend.send_joint_ee_command(_make_command())
        assert backend._parked is True

        backend.start()  # operator re-launches kylos
        assert backend._parked is False
        assert backend._latest_action_monotonic_ns == 0
        assert fake_driver.primed is True

    def test_notify_does_not_advance_on_repeated_header(
        self,
        backend: HardwareManipulatorBackend,
        fake_now: dict,
    ) -> None:
        backend.start()
        # Same header notified repeatedly (LCM subscriber holds the last message). Internal stamp
        # must not advance with later wall-time, so once the threshold elapses the watchdog will
        # still trip on the next send.
        repeating_header = TimestampHeader(monotonic_ns=fake_now["now_ns"], system_ns=0)
        backend.pet_watchdog(repeating_header)
        first_stamp = backend._latest_action_monotonic_ns
        fake_now["now_ns"] += int(1.0 * 1e9)
        backend.pet_watchdog(repeating_header)
        assert backend._latest_action_monotonic_ns == first_stamp


if __name__ == "__main__":
    run_manor_tests()
