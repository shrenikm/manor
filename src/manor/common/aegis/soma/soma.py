"""
Soma LeafSystem: forward kinematics and proprioception assembly.

Subscribes to JointState and EEFState, performs FK on the joint state to
produce an EEF pose + twist, and emits a combined Proprioception message.

The FK computation itself is currently a stub: it emits a default
(identity) EEFPose and a zero EEFTwist. Once the kinematic model loader is
wired in, only ``_compute_eef_pose`` / ``_compute_eef_twist`` should need
to change.
"""

from __future__ import annotations

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.defaults import (
    default_eef_pose,
    default_eef_state,
    default_eef_twist,
    default_joint_state,
    default_proprioception,
    system_time_header,
)
from manor.common.custom_types import FilePath
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.proprioception import Proprioception


class Soma(LeafSystem):
    """
    Assembles Proprioception from joint + EEF state, filling in EEF pose +
    twist via forward kinematics.
    """

    def __init__(self, robot_model_path: FilePath | None, publish_frequency: float) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self._robot_model_path = robot_model_path
        self._publish_frequency = publish_frequency

        self._joint_state_input = self.DeclareAbstractInputPort(
            "joint_state",
            AbstractValue.Make(default_joint_state()),
        )
        self._eef_state_input = self.DeclareAbstractInputPort(
            "eef_state",
            AbstractValue.Make(default_eef_state()),
        )

        self._proprioception_state_index = self.DeclareAbstractState(
            AbstractValue.Make(default_proprioception()),
        )

        self.DeclareAbstractOutputPort(
            "proprioception",
            alloc=lambda: AbstractValue.Make(default_proprioception()),
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
    def robot_model_path(self) -> FilePath | None:
        return self._robot_model_path

    def _calc_proprioception_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._proprioception_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        joint_state: JointState = self._joint_state_input.Eval(context)
        eef_state: EEFState = self._eef_state_input.Eval(context)

        proprioception = Proprioception(
            header=system_time_header(),
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
        return default_eef_pose()

    def _compute_eef_twist(self, joint_state: JointState) -> EEFTwist:
        # TODO: spatial-Jacobian-based twist once the model is wired in.
        del joint_state
        return default_eef_twist()
