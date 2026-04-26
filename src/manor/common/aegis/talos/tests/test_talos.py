"""
Tests for Talos: port shape, periodic publish, backend delegation, FK
assembly, and backend protocol compliance.
"""

from __future__ import annotations

from unittest import mock

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.sim.sim import Sim
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
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.driver import Lite6Driver
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


class _RecordingBackend:
    def __init__(self) -> None:
        self.commands: list[Command] = []
        self.started = False
        self.stopped = False

    def send_command(self, command: Command) -> None:
        self.commands.append(command)

    def read_joint_state(self) -> JointState:
        return JointState.construct_default(num_joints=3)

    def read_eef_state(self) -> EEFState:
        return EEFState.construct_default(num_eef_dofs=1)

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


def _make_lite6_model() -> Lite6Model:
    return Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL)


def _make_talos(**overrides) -> Talos:
    defaults = dict(
        backend=_RecordingBackend(),
        manipulator_model=_make_lite6_model(),
        publish_frequency=100.0,
    )
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

    def test_owns_a_finalized_plant(self) -> None:
        talos = _make_talos()
        assert talos.plant.is_finalized()


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
        talos.GetInputPort(TalosPorts.INPUT_COMMAND).FixValue(context, AbstractValue.Make(Command.construct_default()))

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        proprioception = talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION).Eval(simulator.get_context())
        assert isinstance(proprioception, Proprioception)
        assert isinstance(proprioception.joint_state, JointState)
        assert isinstance(proprioception.eef_state, EEFState)
        assert isinstance(proprioception.eef_pose, EEFPose)
        assert isinstance(proprioception.eef_twist, EEFTwist)
        assert proprioception.header.monotonic_ns > 0


class TestManipulatorBackendProtocolCompliance:
    def test_sim_backend_satisfies_protocol(self) -> None:
        sim = Sim(manipulator_model=_make_lite6_model())
        backend = SimManipulatorBackend(sim=sim, config=SimManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)

    def test_hardware_backend_satisfies_protocol(self) -> None:
        driver = Lite6Driver(model=_make_lite6_model())
        backend = HardwareManipulatorBackend(driver=driver, config=HardwareManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)


class TestSimManipulatorBackend:
    def test_send_command_routes_joint_positions_to_sim(self) -> None:
        sim = mock.MagicMock(spec=Sim)
        backend = SimManipulatorBackend(sim=sim)
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
        command = Command(
            header=TimestampHeader.from_system_time(),
            joint_positions=JointPositions(header=TimestampHeader.from_system_time(), positions=positions),
        )
        backend.send_command(command)
        sim.apply_joint_position_command.assert_called_once()


if __name__ == "__main__":
    run_manor_tests()
