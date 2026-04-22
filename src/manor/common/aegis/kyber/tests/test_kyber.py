"""
Tests for Kyber: message passing, periodic publish cadence, and output hold.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.kyber.kyber import Kyber
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


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
    kyber.GetInputPort("action").FixValue(context, AbstractValue.Make(action))
    kyber.GetInputPort("proprioception").FixValue(context, AbstractValue.Make(proprioception))


def _read_command(kyber: Kyber, context) -> Command:
    return kyber.GetOutputPort("command").Eval(context)


class TestKyberConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            Kyber(publish_frequency=0.0)
        with pytest.raises(ValueError):
            Kyber(publish_frequency=-10.0)

    def test_declares_expected_ports(self) -> None:
        kyber = Kyber(publish_frequency=100.0)
        assert kyber.num_input_ports() == 2
        assert kyber.num_output_ports() == 1
        assert kyber.GetInputPort("action") is not None
        assert kyber.GetInputPort("proprioception") is not None
        assert kyber.GetOutputPort("command") is not None

    def test_stores_publish_frequency(self) -> None:
        kyber = Kyber(publish_frequency=250.0)
        assert kyber.publish_frequency == 250.0


class TestKyberPassthrough:
    def test_action_joint_positions_flow_to_command(self) -> None:
        kyber = Kyber(publish_frequency=100.0)
        context = kyber.CreateDefaultContext()

        positions = np.array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6], dtype=np.float64)
        _fix_inputs(kyber, context, _make_action(positions), _make_proprioception(n_joints=6))

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.05)

        command = _read_command(kyber, simulator.get_context())
        assert isinstance(command, Command)
        assert command.joint_positions is not None
        np.testing.assert_array_equal(command.joint_positions.positions, positions)

    def test_command_header_is_wall_clock(self) -> None:
        import time as _time

        kyber = Kyber(publish_frequency=100.0)
        context = kyber.CreateDefaultContext()
        _fix_inputs(kyber, context, _make_action(np.zeros(3)), _make_proprioception(n_joints=3))

        before_mono = _time.monotonic_ns()
        before_sys = _time.time_ns()
        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.02)
        after_mono = _time.monotonic_ns()
        after_sys = _time.time_ns()

        command = _read_command(kyber, simulator.get_context())
        assert before_mono <= command.header.monotonic_ns <= after_mono
        assert before_sys <= command.header.system_ns <= after_sys


class TestKyberPeriodicCadence:
    def test_command_updates_when_action_changes(self) -> None:
        kyber = Kyber(publish_frequency=100.0)
        context = kyber.CreateDefaultContext()

        first = np.array([1.0, 1.0, 1.0], dtype=np.float64)
        _fix_inputs(kyber, context, _make_action(first), _make_proprioception(n_joints=3))

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.05)
        cmd_first = _read_command(kyber, simulator.get_context())
        np.testing.assert_array_equal(cmd_first.joint_positions.positions, first)

        second = np.array([2.0, -2.0, 3.5], dtype=np.float64)
        kyber.GetInputPort("action").FixValue(simulator.get_context(), AbstractValue.Make(_make_action(second)))
        simulator.AdvanceTo(0.10)
        cmd_second = _read_command(kyber, simulator.get_context())
        np.testing.assert_array_equal(cmd_second.joint_positions.positions, second)

    def test_output_holds_between_ticks(self) -> None:
        # At 10 Hz the period is 0.1s; advancing to 0.25s should trigger exactly
        # three unrestricted updates (at t = 0.0, 0.1, 0.2). The output held
        # between ticks must equal the most-recent tick's value, so a static
        # input produces a stable output across the whole run.
        kyber = Kyber(publish_frequency=10.0)
        context = kyber.CreateDefaultContext()
        positions = np.array([0.7, 0.8], dtype=np.float64)
        _fix_inputs(kyber, context, _make_action(positions), _make_proprioception(n_joints=2))

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.25)

        command = _read_command(kyber, simulator.get_context())
        np.testing.assert_array_equal(command.joint_positions.positions, positions)
