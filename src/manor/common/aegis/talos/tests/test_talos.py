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

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend, HardwareManipulatorBackendConfig
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend, SimManipulatorBackendConfig
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosPorts
from manor.common.definitions.cartesian_state import CartesianState
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.driver import Lite6Driver, Lite6DriverConfig
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


class _RecordingBackend:
    def __init__(self) -> None:
        self.commands: list[JointEECommand] = []
        self.started = False
        self.stopped = False

    def send_joint_ee_command(self, joint_ee_command: JointEECommand) -> None:
        self.commands.append(joint_ee_command)

    def read_joint_state(self) -> JointState:
        return JointState.construct_default(num_joints=3)

    def read_ee_state(self) -> EEState:
        return EEState.construct_default(num_ee_dofs=1)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True


def _make_joint_ee_command(positions: np.ndarray) -> JointEECommand:
    return JointEECommand(
        header=TimestampHeader(monotonic_ns=1, system_ns=2),
        joint_command=JointCommand(
            header=TimestampHeader(monotonic_ns=3, system_ns=4),
            joint_positions=JointPositions(
                header=TimestampHeader(monotonic_ns=5, system_ns=6),
                positions=positions,
            ),
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
        assert talos.GetInputPort(TalosPorts.INPUT_JOINT_EE_COMMAND) is not None
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
        talos.GetInputPort(TalosPorts.INPUT_JOINT_EE_COMMAND).FixValue(
            context, AbstractValue.Make(_make_joint_ee_command(positions))
        )

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        assert len(backend.commands) >= 1
        np.testing.assert_array_equal(backend.commands[-1].joint_command.joint_positions.positions, positions)

    def test_publishes_assembled_proprioception(self) -> None:
        talos = _make_talos()
        context = talos.CreateDefaultContext()
        talos.GetInputPort(TalosPorts.INPUT_JOINT_EE_COMMAND).FixValue(
            context, AbstractValue.Make(JointEECommand.construct_default())
        )

        simulator = Simulator(talos, context)
        simulator.AdvanceTo(0.05)

        proprioception = talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION).Eval(simulator.get_context())
        assert isinstance(proprioception, Proprioception)
        assert isinstance(proprioception.joint_state, JointState)
        assert isinstance(proprioception.cartesian_state, CartesianState)
        assert isinstance(proprioception.ee_state, EEState)
        assert proprioception.header.monotonic_ns > 0


class TestManipulatorBackendProtocolCompliance:
    def test_sim_backend_satisfies_protocol(self) -> None:
        gaia = Gaia(manipulator_model=_make_lite6_model())
        gaia.finalize()
        backend = SimManipulatorBackend(gaia=gaia, config=SimManipulatorBackendConfig())
        assert isinstance(backend, ManipulatorBackend)

    def test_hardware_backend_satisfies_protocol(self) -> None:
        driver_config = Lite6DriverConfig(joint_speed_limit_rad_s=1.0)
        driver = Lite6Driver(model=_make_lite6_model(), config=driver_config)
        backend = HardwareManipulatorBackend(
            driver=driver,
            config=HardwareManipulatorBackendConfig(
                minimum_watchdog_frequency_hz=3.0,
                lite6_driver_config=driver_config,
            ),
        )
        assert isinstance(backend, ManipulatorBackend)


class TestSimManipulatorBackend:
    def test_send_joint_ee_command_routes_joint_positions_to_gaia(self) -> None:
        gaia = mock.MagicMock(spec=Gaia)
        backend = SimManipulatorBackend(gaia=gaia)
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
        joint_ee_command = JointEECommand(
            header=TimestampHeader.from_system_time(),
            joint_command=JointCommand(
                header=TimestampHeader.from_system_time(),
                joint_positions=JointPositions(header=TimestampHeader.from_system_time(), positions=positions),
            ),
        )
        backend.send_joint_ee_command(joint_ee_command)
        gaia.apply_joint_position_command.assert_called_once()

    def test_send_joint_ee_command_routes_ee_positions_to_gaia(self) -> None:
        gaia = mock.MagicMock(spec=Gaia)
        backend = SimManipulatorBackend(gaia=gaia)
        joint_ee_command = JointEECommand(
            header=TimestampHeader.from_system_time(),
            joint_command=JointCommand(
                header=TimestampHeader.from_system_time(),
                joint_positions=JointPositions(
                    header=TimestampHeader.from_system_time(),
                    positions=np.zeros(6, dtype=np.float64),
                ),
            ),
            ee_command=EECommand(
                header=TimestampHeader.from_system_time(),
                ee_positions=EEPositions(
                    header=TimestampHeader.from_system_time(),
                    positions=np.array([0.012], dtype=np.float64),
                ),
            ),
        )
        backend.send_joint_ee_command(joint_ee_command)
        gaia.apply_ee_position_command.assert_called_once()
        gaia.apply_ee_velocity_command.assert_not_called()

    def test_send_joint_ee_command_routes_ee_velocities_to_gaia(self) -> None:
        gaia = mock.MagicMock(spec=Gaia)
        backend = SimManipulatorBackend(gaia=gaia)
        joint_ee_command = JointEECommand(
            header=TimestampHeader.from_system_time(),
            joint_command=JointCommand(
                header=TimestampHeader.from_system_time(),
                joint_positions=JointPositions(
                    header=TimestampHeader.from_system_time(),
                    positions=np.zeros(6, dtype=np.float64),
                ),
            ),
            ee_command=EECommand(
                header=TimestampHeader.from_system_time(),
                ee_velocities=EEVelocities(
                    header=TimestampHeader.from_system_time(),
                    velocities=np.array([0.04], dtype=np.float64),
                ),
            ),
        )
        backend.send_joint_ee_command(joint_ee_command)
        gaia.apply_ee_velocity_command.assert_called_once()
        gaia.apply_ee_position_command.assert_not_called()

    def test_read_ee_state_returns_width_from_gripper_joints(self) -> None:
        gaia = Gaia(manipulator_model=_make_lite6_model())
        gaia.finalize()
        backend = SimManipulatorBackend(gaia=gaia)
        ee_state = backend.read_ee_state()
        # The model's EE DOF count is 1 (parallel gripper opening width).
        assert ee_state.ee_positions.positions.shape == (1,)

    def test_start_snaps_plant_to_prime_plant_positions(self) -> None:
        # Mirror of HardwareManipulatorBackend.start: in sim there's no streaming controller to
        # ramp toward the target, so start snaps the plant context to the model's PRIME plant
        # positions vector. Verifies the model lookup goes through manipulator_model and the call
        # routes to gaia.set_joint_positions with that exact vector.
        model = _make_lite6_model()
        gaia = mock.MagicMock(spec=Gaia)
        gaia.manipulator_model = model
        backend = SimManipulatorBackend(gaia=gaia)
        backend.start()
        gaia.set_joint_positions.assert_called_once()
        called_with = gaia.set_joint_positions.call_args.args[0]
        np.testing.assert_array_equal(called_with, model.get_prime_plant_positions())

    def test_stop_snaps_plant_to_rest_plant_positions(self) -> None:
        model = _make_lite6_model()
        gaia = mock.MagicMock(spec=Gaia)
        gaia.manipulator_model = model
        backend = SimManipulatorBackend(gaia=gaia)
        backend.stop()
        gaia.set_joint_positions.assert_called_once()
        called_with = gaia.set_joint_positions.call_args.args[0]
        np.testing.assert_array_equal(called_with, model.get_rest_plant_positions())


if __name__ == "__main__":
    run_manor_tests()
