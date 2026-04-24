"""
Kyber: the low-level controller.

Consumes Action and Proprioception and produces Command at a fixed rate set by
``publish_frequency``. The output rate is decoupled from the input message
rates: on every periodic tick Kyber samples whatever values currently sit on
its input ports and writes a fresh Command into abstract state. The output
port holds that value until the next tick (zero-order hold).

This initial implementation is a passthrough: it assumes the incoming Action
carries a ``joint_positions`` variant and forwards those positions into the
outgoing Command. A proper controller protocol will replace this.
"""

from __future__ import annotations

from enum import StrEnum

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.utils.defaults import (
    construct_default_action,
    construct_default_command,
    construct_default_proprioception,
    construct_system_time_header,
)


class KyberPorts(StrEnum):
    """
    Named input / output ports exposed by Kyber.
    """

    INPUT_ACTION = "action"
    INPUT_PROPRIOCEPTION = "proprioception"
    OUTPUT_COMMAND = "command"


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
            KyberPorts.INPUT_ACTION,
            AbstractValue.Make(construct_default_action()),
        )
        self._proprioception_input = self.DeclareAbstractInputPort(
            KyberPorts.INPUT_PROPRIOCEPTION,
            AbstractValue.Make(construct_default_proprioception()),
        )

        self._command_state_index = self.DeclareAbstractState(AbstractValue.Make(construct_default_command()))

        self.DeclareAbstractOutputPort(
            KyberPorts.OUTPUT_COMMAND,
            alloc=lambda: AbstractValue.Make(construct_default_command()),
            calc=self._calc_command_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._command_state_index)},
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

        command = Command(header=construct_system_time_header(), joint_positions=action.joint_positions)
        state.get_mutable_abstract_state(self._command_state_index).set_value(command)
        return EventStatus.Succeeded()
