"""
Kyber: the low-level controller.

Consumes Action and Proprioception and produces Command at a fixed rate
set by ``publish_frequency``. Kyber owns its own ``MultibodyPlant``
built from the supplied ``IManipulatorModel`` (the slot for future
diff-IK / trajectory tracking); each aegis sub-system that needs a
plant constructs its own rather than sharing one across systems.

This phase's implementation is a stub: regardless of the incoming
Action variant, Kyber emits a zero ``JointVelocities`` Command sized to
the manipulator's actuated DOF count. The full controller (joint-space
+ task-space tracking, IK on EEF actions) lives in a follow-up plan.
"""

from __future__ import annotations

from enum import StrEnum

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel


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

    Owns its own ``MultibodyPlant`` (via ``manipulator_model``) so future
    diff-IK / trajectory tracking work has somewhere to live without
    sharing state with Talos or Sim.
    """

    def __init__(
        self,
        manipulator_model: IManipulatorModel,
        publish_frequency: float,
    ) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self._manipulator_model = manipulator_model
        self._publish_frequency = publish_frequency
        self._plant = self._build_plant(manipulator_model)

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

    @property
    def publish_frequency(self) -> float:
        return self._publish_frequency

    @property
    def manipulator_model(self) -> IManipulatorModel:
        return self._manipulator_model

    @property
    def plant(self) -> MultibodyPlant:
        return self._plant

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
        command = context.get_abstract_state(self._command_state_index).get_value()
        output.set_value(command)

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        # Stub: ignore the action and emit a zero-velocity command sized
        # to the manipulator's actuated DOF count. Proprioception is
        # held as a future input for closed-loop control.
        self._action_input.Eval(context)
        self._proprioception_input.Eval(context)

        header = TimestampHeader.from_system_time()
        command = Command(
            header=header,
            joint_velocities=JointVelocities(
                header=header,
                velocities=np.zeros(self._manipulator_model.get_num_dof(), dtype=np.float64),
            ),
        )
        state.get_mutable_abstract_state(self._command_state_index).set_value(command)
        return EventStatus.Succeeded()
