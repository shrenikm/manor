"""
Tests for Kyber: port shape, plant ownership, and Controller dispatch.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.kyber.controllers import ZeroVelocityController
from manor.common.aegis.kyber.kyber import Controller, Kyber, KyberPorts
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_lite6() -> Lite6Model:
    return Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL)


def _make_kyber(**overrides) -> Kyber:
    defaults = dict(
        controller=ZeroVelocityController(num_dof=LITE6_ARM_DOF),
        manipulator_model=_make_lite6(),
        publish_frequency=100.0,
    )
    defaults.update(overrides)
    return Kyber(**defaults)


def _make_action(positions: np.ndarray) -> Action:
    return Action(
        header=TimestampHeader(monotonic_ns=1, system_ns=2),
        joint_positions=JointPositions(
            header=TimestampHeader(monotonic_ns=3, system_ns=4),
            positions=positions,
        ),
    )


def _make_proprioception(n_joints: int) -> Proprioception:
    return Proprioception(
        header=TimestampHeader(monotonic_ns=5, system_ns=6),
        joint_state=JointState(
            header=TimestampHeader(monotonic_ns=7, system_ns=8),
            joint_positions=JointPositions(
                header=TimestampHeader(monotonic_ns=9, system_ns=10),
                positions=np.zeros(n_joints, dtype=np.float64),
            ),
            joint_velocities=JointVelocities(
                header=TimestampHeader(monotonic_ns=11, system_ns=12),
                velocities=np.zeros(n_joints, dtype=np.float64),
            ),
        ),
    )


def _fix_inputs(kyber: Kyber, context, action: Action, proprioception: Proprioception) -> None:
    kyber.GetInputPort(KyberPorts.INPUT_ACTION).FixValue(context, AbstractValue.Make(action))
    kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION).FixValue(context, AbstractValue.Make(proprioception))


def _read_command(kyber: Kyber, context) -> Command:
    return kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND).Eval(context)


class _RecordingController:
    def __init__(self, canned_command: Command) -> None:
        self.canned = canned_command
        self.calls: list[tuple[Action, Proprioception]] = []

    def step(self, action: Action, proprioception: Proprioception) -> Command:
        self.calls.append((action, proprioception))
        return self.canned


class TestKyberConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            _make_kyber(publish_frequency=0.0)
        with pytest.raises(ValueError):
            _make_kyber(publish_frequency=-10.0)

    def test_declares_expected_ports(self) -> None:
        kyber = _make_kyber()
        assert kyber.num_input_ports() == 2
        assert kyber.num_output_ports() == 1
        assert kyber.GetInputPort(KyberPorts.INPUT_ACTION) is not None
        assert kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION) is not None
        assert kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND) is not None

    def test_owns_a_finalized_plant(self) -> None:
        kyber = _make_kyber()
        assert kyber.plant.is_finalized()

    def test_stores_publish_frequency(self) -> None:
        kyber = _make_kyber(publish_frequency=250.0)
        assert kyber.publish_frequency == 250.0


class TestKyberControllerDispatch:
    def test_periodic_update_calls_controller_with_inputs(self) -> None:
        canned = Command(
            header=TimestampHeader.from_system_time(),
            joint_velocities=JointVelocities(
                header=TimestampHeader.from_system_time(),
                velocities=np.zeros(LITE6_ARM_DOF),
            ),
        )
        controller = _RecordingController(canned)
        kyber = _make_kyber(controller=controller)
        context = kyber.CreateDefaultContext()
        _fix_inputs(
            kyber,
            context,
            _make_action(np.zeros(LITE6_ARM_DOF)),
            _make_proprioception(n_joints=LITE6_ARM_DOF),
        )

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.05)

        assert len(controller.calls) >= 1
        last_action, last_proprioception = controller.calls[-1]
        assert isinstance(last_action, Action)
        assert isinstance(last_proprioception, Proprioception)

    def test_zero_velocity_controller_emits_zero_velocity_command(self) -> None:
        kyber = _make_kyber()
        context = kyber.CreateDefaultContext()
        _fix_inputs(
            kyber,
            context,
            _make_action(np.array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.7, -0.8], dtype=np.float64)),
            _make_proprioception(n_joints=LITE6_ARM_DOF),
        )

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.05)

        command = _read_command(kyber, simulator.get_context())
        assert command.joint_velocities is not None
        np.testing.assert_array_equal(command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF))


class TestZeroVelocityController:
    def test_is_a_controller(self) -> None:
        assert isinstance(ZeroVelocityController(num_dof=LITE6_ARM_DOF), Controller)

    def test_emits_zero_velocity_regardless_of_inputs(self) -> None:
        controller = ZeroVelocityController(num_dof=LITE6_ARM_DOF)
        action = _make_action(np.full(LITE6_ARM_DOF, 0.5))
        proprioception = _make_proprioception(n_joints=LITE6_ARM_DOF)
        command = controller.step(action, proprioception)
        assert command.joint_velocities is not None
        np.testing.assert_array_equal(command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF))


if __name__ == "__main__":
    run_manor_tests()
