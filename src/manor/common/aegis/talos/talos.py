"""
Talos LeafSystem: sends commands to the robot and publishes its state.

On each periodic tick Talos (a) forwards the latest Command on its input port
to the backend, and (b) polls joint + EEF state from the backend and writes
them into abstract state. Output ports are zero-order holds of that state.

Start / end hooks for going to a safe configuration before and after policy
execution are declared on the backend protocol but intentionally not wired
into Drake events yet -- they'll be triggered by higher-level orchestration.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.defaults import (
    default_command,
    default_eef_state,
    default_joint_state,
)
from manor.common.definitions.command import Command
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.joint_state import JointState


@runtime_checkable
class ManipulatorBackend(Protocol):
    """
    Protocol for a manipulator actuation-and-state interface.

    Exactly one backend owns the robot's actual state at a time. Talos
    drives the backend on every tick by calling ``send_command`` and reads
    state back via ``read_joint_state`` / ``read_eef_state``.
    """

    def send_command(self, command: Command) -> None: ...

    def read_joint_state(self) -> JointState: ...

    def read_eef_state(self) -> EEFState: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...


class Talos(LeafSystem):
    """
    Bridge between the Aegis graph and the manipulator. Consumes Command,
    publishes JointState and EEFState.
    """

    def __init__(self, backend: ManipulatorBackend, publish_frequency: float) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self._backend = backend
        self._publish_frequency = publish_frequency

        self._command_input = self.DeclareAbstractInputPort(
            "command",
            AbstractValue.Make(default_command()),
        )

        self._joint_state_index = self.DeclareAbstractState(AbstractValue.Make(default_joint_state()))
        self._eef_state_index = self.DeclareAbstractState(AbstractValue.Make(default_eef_state()))

        self.DeclareAbstractOutputPort(
            "joint_state",
            alloc=lambda: AbstractValue.Make(default_joint_state()),
            calc=self._calc_joint_state_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._joint_state_index)},
        )
        self.DeclareAbstractOutputPort(
            "eef_state",
            alloc=lambda: AbstractValue.Make(default_eef_state()),
            calc=self._calc_eef_state_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._eef_state_index)},
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    @property
    def publish_frequency(self) -> float:
        return self._publish_frequency

    @property
    def backend(self) -> ManipulatorBackend:
        return self._backend

    def _calc_joint_state_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._joint_state_index).get_value())

    def _calc_eef_state_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._eef_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        command: Command = self._command_input.Eval(context)
        self._backend.send_command(command)
        state.get_mutable_abstract_state(self._joint_state_index).set_value(self._backend.read_joint_state())
        state.get_mutable_abstract_state(self._eef_state_index).set_value(self._backend.read_eef_state())
        return EventStatus.Succeeded()
