"""
Tests for Kyber: port shape, plant ownership, and the zero-velocity
stub controller.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.kyber.kyber import Kyber, KyberPorts
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
    defaults = dict(manipulator_model=_make_lite6(), publish_frequency=100.0)
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


class TestKyberZeroVelocityStub:
    def test_emits_zero_velocity_command_for_arm_dof(self) -> None:
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

    def test_command_header_is_system_time(self) -> None:
        import time as _time

        kyber = _make_kyber()
        context = kyber.CreateDefaultContext()
        _fix_inputs(
            kyber,
            context,
            _make_action(np.zeros(LITE6_ARM_DOF)),
            _make_proprioception(n_joints=LITE6_ARM_DOF),
        )

        before_mono = _time.monotonic_ns()
        before_sys = _time.time_ns()
        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.02)
        after_mono = _time.monotonic_ns()
        after_sys = _time.time_ns()

        command = _read_command(kyber, simulator.get_context())
        assert before_mono <= command.header.monotonic_ns <= after_mono
        assert before_sys <= command.header.system_ns <= after_sys


if __name__ == "__main__":
    run_manor_tests()
