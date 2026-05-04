"""
Tests for StaleCommandWatchdog: port shape, periodic notify forwarding,
and integration with HardwareManipulatorBackend.
"""

from __future__ import annotations

import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.talos.stale_command_watchdog import StaleCommandWatchdog, StaleCommandWatchdogPorts
from manor.common.definitions.action import Action
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests


class _RecordingBackend:
    """
    Stand-in for HardwareManipulatorBackend that just records every pet_watchdog call.
    """

    def __init__(self) -> None:
        self.headers: list[TimestampHeader] = []

    def pet_watchdog(self, header: TimestampHeader) -> None:
        self.headers.append(header)


def _make_action(monotonic_ns: int = 100, system_ns: int = 200) -> Action:
    header = TimestampHeader(monotonic_ns=monotonic_ns, system_ns=system_ns)
    return Action(
        header=header,
        joint_command=JointCommand.construct_default(num_joints=6),
    )


class TestConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        backend = _RecordingBackend()
        with pytest.raises(ValueError):
            StaleCommandWatchdog(backend=backend, tick_frequency_hz=0.0)

    def test_declares_expected_ports(self) -> None:
        backend = _RecordingBackend()
        watchdog = StaleCommandWatchdog(backend=backend, tick_frequency_hz=100.0)
        assert watchdog.num_input_ports() == 1
        assert watchdog.num_output_ports() == 0
        assert watchdog.GetInputPort(StaleCommandWatchdogPorts.INPUT_ACTION) is not None


class TestPeriodicNotify:
    def test_forwards_action_header_to_backend(self) -> None:
        backend = _RecordingBackend()
        watchdog = StaleCommandWatchdog(backend=backend, tick_frequency_hz=200.0)

        action = _make_action(monotonic_ns=12_345, system_ns=67_890)
        watchdog.GetInputPort(StaleCommandWatchdogPorts.INPUT_ACTION).FixValue(
            watchdog.CreateDefaultContext(),
            AbstractValue.Make(action),
        )

        # Drive a Simulator past the first tick boundary so the periodic update fires at least once.
        # The watchdog declares its event at offset 0 and period 1 / freq, so AdvanceTo(2 * period)
        # fires the event reliably (>=1 tick).
        simulator = Simulator(watchdog)
        simulator.get_mutable_context()  # ensure the context backing FixValue is also the simulator's
        # Re-fix on the simulator's own context (FixValue above used a throwaway default context).
        watchdog.GetInputPort(StaleCommandWatchdogPorts.INPUT_ACTION).FixValue(
            simulator.get_mutable_context(),
            AbstractValue.Make(action),
        )
        simulator.Initialize()
        simulator.AdvanceTo(0.02)

        assert len(backend.headers) > 0
        assert backend.headers[-1] == action.header


if __name__ == "__main__":
    run_manor_tests()
