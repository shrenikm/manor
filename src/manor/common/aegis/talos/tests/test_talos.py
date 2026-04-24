"""
Tests for Talos: port shape, periodic publish, backend delegation, FK
assembly, and backend protocol compliance.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend, HardwareManipulatorBackendConfig
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend, SimManipulatorBackendConfig
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosPorts
from manor.common.definitions.command import Command
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.defaults import (
    construct_default_command,
    construct_default_eef_state,
    construct_default_joint_state,
)
from manor.common.testing_utils import run_manor_tests


class _RecordingBackend:
    def __init__(self) -> None:
        self.commands: list[Command] = []
        self.started = False
        self.stopped = False

    def send_command(self, command: Command) -> None:
        self.commands.append(command)

    def read_joint_state(self) -> JointState:
        return construct_default_joint_state(num_joints=3)

    def read_eef_state(self) -> EEFState:
        return construct_default_eef_state(num_eef_dofs=1)

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


def _make_talos(**overrides) -> Talos:
    defaults = dict(backend=_RecordingBackend(), robot_model_path=None, publish_frequency=100.0)
    defaults.update(overrides)
    return Talos(**defaults)


class TestTalosConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            _make_talos(publish_frequency=0.0)

    def test_declares_expected_ports(self) -> None:
        talos = _make_talos(publish_frequency=200.0)
        assert talos.num_input_ports() == 1
        assert talos.num_output_ports() == 1
        assert talos.GetInputPort(TalosPorts.INPUT_COMMAND) is not None
        assert talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION) is not None


class TestTalosPeriodic:
    def test_forwards_commands_to_backend(self) -> None:
        backend = _RecordingBackend()
        talos = _make_talos(backend=backend)
        context = talos.CreateDefaultContext()
        positions = np.array([0.1, 0.2, 0.3], dtype=np.float64)
        talos.GetInputPort(TalosPorts.INPUT_COMMAND).FixValue(context, AbstractValue.Make(_make_command(positions)))

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        assert len(backend.commands) >= 1
        np.testing.assert_array_equal(backend.commands[-1].joint_positions.positions, positions)

    def test_publishes_assembled_proprioception(self) -> None:
        talos = _make_talos()
        context = talos.CreateDefaultContext()
        talos.GetInputPort(TalosPorts.INPUT_COMMAND).FixValue(context, AbstractValue.Make(construct_default_command()))

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        proprioception = talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION).Eval(simulator.get_context())
        assert isinstance(proprioception, Proprioception)
        assert isinstance(proprioception.joint_state, JointState)
        assert isinstance(proprioception.eef_state, EEFState)
        assert isinstance(proprioception.eef_pose, EEFPose)
        assert isinstance(proprioception.eef_twist, EEFTwist)
        # Header is stamped via system time -- nonzero monotonic_ns confirms a
        # real tick ran (default construction puts this at 0).
        assert proprioception.header.monotonic_ns > 0


class TestManipulatorBackendProtocolCompliance:
    def test_sim_backend_satisfies_protocol(self) -> None:
        backend = SimManipulatorBackend(config=SimManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)

    def test_hardware_backend_satisfies_protocol(self) -> None:
        backend = HardwareManipulatorBackend(config=HardwareManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)


if __name__ == "__main__":
    run_manor_tests()
