"""
Talos LeafSystem: sends commands to the robot and publishes its full
proprioception state.

On each periodic tick Talos:
  1. forwards the latest Command on its input port to the backend,
  2. polls the backend for the current joint + EEF state,
  3. runs forward kinematics on the joint state to compute EEF pose + twist,
  4. assembles a Proprioception message and writes it into abstract state.

The output port is a zero-order hold of that state. The FK computation is
currently a stub (identity pose, zero twist); once the kinematic model
loader is wired in, only ``_compute_eef_pose`` / ``_compute_eef_twist``
need to change.

Start / end hooks for going to a safe configuration before and after policy
execution are declared on the backend protocol but intentionally not wired
into Drake events yet -- they'll be triggered by higher-level orchestration.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Protocol, runtime_checkable

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.custom_types import FilePath
from manor.common.definitions.command import Command
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader


class TalosPorts(StrEnum):
    """
    Named input / output ports exposed by Talos.
    """

    INPUT_COMMAND = "command"
    OUTPUT_PROPRIOCEPTION = "proprioception"


@runtime_checkable
class ManipulatorBackend(Protocol):
    """
    Protocol for a manipulator actuation-and-state interface.

    Exactly one backend owns the robot's actual state at a time. Talos drives
    the backend on every tick by calling ``send_command`` and reads state
    back via ``read_joint_state`` / ``read_eef_state``. Forward kinematics is
    Talos's responsibility, not the backend's.
    """

    def send_command(self, command: Command) -> None: ...

    def read_joint_state(self) -> JointState: ...

    def read_eef_state(self) -> EEFState: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...


class Talos(LeafSystem):
    """
    Bridge between the Aegis graph and the manipulator. Consumes Command,
    publishes Proprioception.
    """

    def __init__(
        self,
        backend: ManipulatorBackend,
        robot_model_path: FilePath | None,
        publish_frequency: float,
    ) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self._backend = backend
        self._robot_model_path = robot_model_path
        self._publish_frequency = publish_frequency

        self._command_input = self.DeclareAbstractInputPort(
            TalosPorts.INPUT_COMMAND,
            AbstractValue.Make(Command.construct_default()),
        )

        self._proprioception_state_index = self.DeclareAbstractState(
            AbstractValue.Make(Proprioception.construct_default()),
        )

        self.DeclareAbstractOutputPort(
            TalosPorts.OUTPUT_PROPRIOCEPTION,
            alloc=lambda: AbstractValue.Make(Proprioception.construct_default()),
            calc=self._calc_proprioception_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._proprioception_state_index)},
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

    @property
    def robot_model_path(self) -> FilePath | None:
        return self._robot_model_path

    def _calc_proprioception_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._proprioception_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        command: Command = self._command_input.Eval(context)
        self._backend.send_command(command)

        joint_state = self._backend.read_joint_state()
        eef_state = self._backend.read_eef_state()

        proprioception = Proprioception(
            header=TimestampHeader.from_system_time(),
            joint_state=joint_state,
            eef_state=eef_state,
            eef_pose=self._compute_eef_pose(joint_state),
            eef_twist=self._compute_eef_twist(joint_state),
        )
        state.get_mutable_abstract_state(self._proprioception_state_index).set_value(proprioception)
        return EventStatus.Succeeded()

    def _compute_eef_pose(self, joint_state: JointState) -> EEFPose:
        # TODO: load the kinematic model from ``self._robot_model_path`` and run FK.
        del joint_state
        return EEFPose.construct_default()

    def _compute_eef_twist(self, joint_state: JointState) -> EEFTwist:
        # TODO: spatial-Jacobian-based twist once the model is wired in.
        del joint_state
        return EEFTwist.construct_default()
