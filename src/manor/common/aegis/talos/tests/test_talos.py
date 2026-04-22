"""
Tests for Talos: port shape, periodic publish, backend delegation, and
backend protocol compliance.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.defaults import default_command, default_eef_state, default_joint_state
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend, HardwareManipulatorBackendConfig
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend, SimManipulatorBackendConfig
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos
from manor.common.definitions.command import Command
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests


class _RecordingBackend:
    def __init__(self) -> None:
        self.commands: list[Command] = []
        self.started = False
        self.stopped = False

    def send_command(self, command: Command) -> None:
        self.commands.append(command)

    def read_joint_state(self) -> JointState:
        return default_joint_state(num_joints=3)

    def read_eef_state(self) -> EEFState:
        return default_eef_state(num_eef_dofs=1)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True


def _make_command(positions: np.ndarray) -> Command:
    return Command(
        header=TimestampHeader(monotonic_ns=1, system_ns=2),
        joint_positions=JointPositions(
            header=TimestampHeader(monotonic_ns=3, system_ns=4),
            positions=positions,
        ),
    )


class TestTalosConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            Talos(backend=_RecordingBackend(), publish_frequency=0.0)

    def test_declares_expected_ports(self) -> None:
        talos = Talos(backend=_RecordingBackend(), publish_frequency=200.0)
        assert talos.num_input_ports() == 1
        assert talos.num_output_ports() == 2
        assert talos.GetInputPort("command") is not None
        assert talos.GetOutputPort("joint_state") is not None
        assert talos.GetOutputPort("eef_state") is not None


class TestTalosPeriodic:
    def test_forwards_commands_to_backend(self) -> None:
        backend = _RecordingBackend()
        talos = Talos(backend=backend, publish_frequency=100.0)
        context = talos.CreateDefaultContext()
        positions = np.array([0.1, 0.2, 0.3], dtype=np.float64)
        talos.GetInputPort("command").FixValue(context, AbstractValue.Make(_make_command(positions)))

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        assert len(backend.commands) >= 1
        np.testing.assert_array_equal(backend.commands[-1].joint_positions.positions, positions)

    def test_publishes_backend_state_on_output(self) -> None:
        talos = Talos(backend=_RecordingBackend(), publish_frequency=100.0)
        context = talos.CreateDefaultContext()
        talos.GetInputPort("command").FixValue(context, AbstractValue.Make(default_command()))

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        joint_state = talos.GetOutputPort("joint_state").Eval(simulator.get_context())
        eef_state = talos.GetOutputPort("eef_state").Eval(simulator.get_context())
        assert isinstance(joint_state, JointState)
        assert isinstance(eef_state, EEFState)


class TestManipulatorBackendProtocolCompliance:
    def test_sim_backend_satisfies_protocol(self) -> None:
        backend = SimManipulatorBackend(config=SimManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)

    def test_hardware_backend_satisfies_protocol(self) -> None:
        backend = HardwareManipulatorBackend(config=HardwareManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)


if __name__ == "__main__":
    run_manor_tests()
