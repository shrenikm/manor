"""
Kyber low-level controller system.

Kyber is a Drake LeafSystem that consumes Action and Proprioception messages
and publishes Command messages at a fixed rate set by ``publish_frequency``.
The output rate is decoupled from the input message rates: on every periodic
tick, Kyber samples whatever values currently sit on its input ports and
produces a fresh Command, which the output port then holds until the next
tick (zero-order hold).

This initial implementation is a passthrough: it assumes the incoming Action
carries a ``joint_positions`` variant and forwards those positions into the
outgoing Command. A proper controller protocol will replace this later.
"""

from __future__ import annotations

import time

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


def _zero_header() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=0, system_ns=0)


def _system_time_header() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=time.monotonic_ns(), system_ns=time.time_ns())


def _model_joint_positions() -> JointPositions:
    return JointPositions(header=_zero_header(), positions=np.zeros(0, dtype=np.float64))


def _model_action() -> Action:
    return Action(header=_zero_header(), joint_positions=_model_joint_positions())


def _model_proprioception() -> Proprioception:
    return Proprioception(
        header=_zero_header(),
        joint_state=JointState(
            header=_zero_header(),
            joint_positions=_model_joint_positions(),
            joint_velocities=JointVelocities(header=_zero_header(), velocities=np.zeros(0, dtype=np.float64)),
        ),
    )


def _model_command() -> Command:
    return Command(header=_zero_header(), joint_positions=_model_joint_positions())


class Kyber(LeafSystem):
    """
    Low-level controller LeafSystem that turns Actions into Commands.
    """

    def __init__(self, publish_frequency: float) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self._publish_frequency = publish_frequency

        self._action_input = self.DeclareAbstractInputPort(
            "action",
            AbstractValue.Make(_model_action()),
        )
        self._proprioception_input = self.DeclareAbstractInputPort(
            "proprioception",
            AbstractValue.Make(_model_proprioception()),
        )

        self._command_state_index = self.DeclareAbstractState(AbstractValue.Make(_model_command()))

        self.DeclareAbstractOutputPort(
            "command",
            alloc=lambda: AbstractValue.Make(_model_command()),
            calc=self._calc_command_output,
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    @property
    def publish_frequency(self) -> float:
        return self._publish_frequency

    def _calc_command_output(self, context: Context, output: AbstractValue) -> None:
        command = context.get_abstract_state(self._command_state_index).get_value()
        output.set_value(command)

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        action: Action = self._action_input.Eval(context)
        # Proprioception is wired into the interface but unused by this passthrough controller.
        self._proprioception_input.Eval(context)

        command = Command(header=_system_time_header(), joint_positions=action.joint_positions)
        state.get_mutable_abstract_state(self._command_state_index).set_value(command)
        return EventStatus.Succeeded()
