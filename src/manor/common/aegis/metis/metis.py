"""
Metis LeafSystem: runs a policy on observations to produce actions.

On every periodic tick Metis:
  1. reads the latest Proprioception + RGB + Depth from its input ports,
  2. assembles them into an Observation,
  3. calls ``policy.step(observation)`` to produce an Action,
  4. writes the Action into abstract state.

The output port is a zero-order hold on that state. The Policy protocol
is what downstream algorithm implementations (motion planners,
diffusion policies, VLAs) plug into.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, runtime_checkable

import attr
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader


class MetisPorts(StrEnum):
    """
    Named input / output ports exposed by Metis.
    """

    INPUT_PROPRIOCEPTION = "proprioception"
    INPUT_RGB_IMAGE = "rgb_image"
    INPUT_DEPTH_IMAGE = "depth_image"
    OUTPUT_ACTION = "action"


@runtime_checkable
class Policy(Protocol):
    """
    Protocol for an observation-to-action policy.

    Concrete implementations may be purely functional (classical
    planners, trajopt) or stateful (learned policies with internal
    recurrence); the ``step`` interface accommodates both.
    """

    def step(self, observation: Observation) -> Action: ...


@attr.frozen
class MetisConfig:
    """
    Metis sub-system configuration. ``policy`` defaults to a
    ``ZeroVelocityPolicy`` sized to the manipulator's DOF count when
    left as ``None`` (resolved by the aegis builder). ``SYSTEM_NAME``
    is the name applied to the Metis LeafSystem in the diagram.
    """

    SYSTEM_NAME: ClassVar[str] = "metis"

    publish_frequency_hz: float = 10.0
    policy: Policy | None = None


class Metis(LeafSystem):
    """
    Runs a Policy on Observations and publishes Actions.
    """

    def __init__(self, policy: Policy, publish_frequency: float) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self.policy = policy
        self.publish_frequency = publish_frequency

        self._proprioception_input = self.DeclareAbstractInputPort(
            MetisPorts.INPUT_PROPRIOCEPTION,
            AbstractValue.Make(Proprioception.construct_default()),
        )
        self._rgb_input = self.DeclareAbstractInputPort(
            MetisPorts.INPUT_RGB_IMAGE,
            AbstractValue.Make(RGBImageData.construct_default()),
        )
        self._depth_input = self.DeclareAbstractInputPort(
            MetisPorts.INPUT_DEPTH_IMAGE,
            AbstractValue.Make(DepthImageData.construct_default()),
        )

        self._action_state_index = self.DeclareAbstractState(AbstractValue.Make(Action.construct_default()))

        self.DeclareAbstractOutputPort(
            MetisPorts.OUTPUT_ACTION,
            alloc=lambda: AbstractValue.Make(Action.construct_default()),
            calc=self._calc_action_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._action_state_index)},
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    def _calc_action_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._action_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        proprioception: Proprioception = self._proprioception_input.Eval(context)
        rgb: RGBImageData = self._rgb_input.Eval(context)
        # Depth is wired for future use; not yet packed into Observation by the top-level schema.
        self._depth_input.Eval(context)

        observation = Observation(
            header=TimestampHeader.from_system_time(),
            proprioception=proprioception,
            rgb_image=rgb,
            rgbd_image=None,
        )
        action = self.policy.step(observation)
        state.get_mutable_abstract_state(self._action_state_index).set_value(action)
        return EventStatus.Succeeded()
