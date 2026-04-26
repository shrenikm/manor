"""
Kyber: the low-level controller LeafSystem.

Consumes Action and Proprioception and produces Command at a fixed rate
set by ``publish_frequency``. The actual control law lives in a
``Controller`` implementation (mirroring how Metis takes a ``Policy``);
Kyber is just the periodic harness that pushes inputs through it.

Kyber owns its own ``MultibodyPlant`` built from the supplied
``IManipulatorModel`` -- the slot for future diff-IK / trajectory
tracking. Each aegis sub-system that needs a plant constructs its own
rather than sharing one across systems.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, runtime_checkable

import attr
from pydrake.common.value import AbstractValue
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.proprioception import Proprioception
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel


class KyberPorts(StrEnum):
    """
    Named input / output ports exposed by Kyber.
    """

    INPUT_ACTION = "action"
    INPUT_PROPRIOCEPTION = "proprioception"
    OUTPUT_COMMAND = "command"


@runtime_checkable
class Controller(Protocol):
    """
    Protocol for a (action, proprioception) -> command controller.

    Concrete implementations may be purely functional (passthrough,
    zero-velocity stub) or stateful (PID with integrator state, MPC
    with internal solvers); the ``step`` interface accommodates both.
    """

    def step(self, action: Action, proprioception: Proprioception) -> Command: ...


@attr.frozen
class KyberConfig:
    """
    Kyber sub-system configuration. ``controller`` defaults to a
    ``ZeroVelocityController`` sized to the manipulator's DOF count
    when left as ``None`` (resolved by the aegis builder).
    ``SYSTEM_NAME`` is the name applied to the Kyber LeafSystem in the
    diagram.
    """

    SYSTEM_NAME: ClassVar[str] = "kyber"

    publish_frequency_hz: float = 500.0
    controller: Controller | None = None


class Kyber(LeafSystem):
    """
    Low-level controller LeafSystem that runs a ``Controller`` on
    (action, proprioception) inputs and publishes a Command output.
    """

    def __init__(
        self,
        controller: Controller,
        manipulator_model: IManipulatorModel,
        publish_frequency: float,
    ) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self.controller = controller
        self.manipulator_model = manipulator_model
        self.publish_frequency = publish_frequency
        self.plant = self._build_plant(manipulator_model)

        self._action_input = self.DeclareAbstractInputPort(
            KyberPorts.INPUT_ACTION,
            AbstractValue.Make(Action.construct_default()),
        )
        self._proprioception_input = self.DeclareAbstractInputPort(
            KyberPorts.INPUT_PROPRIOCEPTION,
            AbstractValue.Make(Proprioception.construct_default()),
        )

        self._command_state_index = self.DeclareAbstractState(AbstractValue.Make(Command.construct_default()))

        self.DeclareAbstractOutputPort(
            KyberPorts.OUTPUT_COMMAND,
            alloc=lambda: AbstractValue.Make(Command.construct_default()),
            calc=self._calc_command_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._command_state_index)},
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    @staticmethod
    def _build_plant(manipulator_model: IManipulatorModel) -> MultibodyPlant:
        # A bare control-only plant: no scene graph, no env. Just enough
        # for downstream IK / spatial-Jacobian work to slot in.
        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        add_robot_models_to_package_map(parser.package_map())
        model_index = parser.AddModels(manipulator_model.get_description_filepath())[0]
        base_frame = plant.GetFrameByName(manipulator_model.get_base_frame_name(), model_index)
        plant.WeldFrames(plant.world_frame(), base_frame)
        plant.Finalize()
        return plant

    def _calc_command_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._command_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        action: Action = self._action_input.Eval(context)
        proprioception: Proprioception = self._proprioception_input.Eval(context)
        command = self.controller.step(action, proprioception)
        state.get_mutable_abstract_state(self._command_state_index).set_value(command)
        return EventStatus.Succeeded()
