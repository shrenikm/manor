"""
Kyber: the low-level controller LeafSystem.

Consumes Action and Proprioception and produces JointEECommand at a fixed rate set by
publish_frequency. The actual control law lives in a KyberController implementation (mirroring how
Metis takes a MetisPolicy); Kyber is just the periodic harness that pushes inputs through it.

Kyber owns no Drake plant of its own. Controllers that need a MultibodyPlant (diff-IK, joint-space
PID with FK lookups, etc.) build one inside their own constructor from the IManipulatorModel the
manager forwards to them. Keeping plant ownership controller-side means stub controllers don't pay
any plant-construction cost.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Self

import attr
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.kyber.controllers.controller_manager import (
    KyberController,
    KyberControllerConfigBase,
)
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.action import Action
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.proprioception import Proprioception


class KyberPorts(StrEnum):
    """
    Named input / output ports exposed by Kyber.
    """

    INPUT_ACTION = "action"
    INPUT_PROPRIOCEPTION = "proprioception"
    OUTPUT_JOINT_EE_COMMAND = "joint_ee_command"


@attr.frozen
class KyberConfig:
    """
    Kyber sub-system configuration.

    controller_config is required: it pins which control law runs on the robot. Defaulting it would
    silently swap behaviour at the most consequential layer of the stack, so aegis refuses to build
    without an explicit choice. SYSTEM_NAME is the name applied to the Kyber LeafSystem in the
    diagram.
    """

    SYSTEM_NAME: ClassVar[str] = "kyber"

    controller_config: KyberControllerConfigBase
    publish_frequency_hz: float = 500.0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        """
        Parse the kyber_config: block of an aegis YAML.
        """
        return cls(**parse_attrs_yaml(cls, d, "kyber_config"))


class Kyber(LeafSystem):
    """
    Low-level controller LeafSystem that runs a KyberController on (action, proprioception) inputs
    and publishes a JointEECommand output.
    """

    def __init__(self, controller: KyberController, publish_frequency: float) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self.controller = controller
        self.publish_frequency = publish_frequency

        self._action_input = self.DeclareAbstractInputPort(
            KyberPorts.INPUT_ACTION,
            AbstractValue.Make(Action.construct_default()),
        )
        self._proprioception_input = self.DeclareAbstractInputPort(
            KyberPorts.INPUT_PROPRIOCEPTION,
            AbstractValue.Make(Proprioception.construct_default()),
        )

        self._joint_ee_command_state_index = self.DeclareAbstractState(
            AbstractValue.Make(JointEECommand.construct_default())
        )

        self.DeclareAbstractOutputPort(
            KyberPorts.OUTPUT_JOINT_EE_COMMAND,
            alloc=lambda: AbstractValue.Make(JointEECommand.construct_default()),
            calc=self._calc_joint_ee_command_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._joint_ee_command_state_index)},
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    def _calc_joint_ee_command_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._joint_ee_command_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        action: Action = self._action_input.Eval(context)
        proprioception: Proprioception = self._proprioception_input.Eval(context)
        joint_ee_command = self.controller.step(action, proprioception)
        state.get_mutable_abstract_state(self._joint_ee_command_state_index).set_value(joint_ee_command)
        return EventStatus.Succeeded()
