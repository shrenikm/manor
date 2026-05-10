"""
GripperOpenClosePolicy: cycles the end-effector between its
EE-position lower and upper kinematic limits, holding each for a
configured duration. Repeats for num_cycles cycles, then settles at
the lower-limit setpoint.

The setpoints come from IManipulatorModel.get_ee_position_limits, so
the policy is generic over robots: a parallel gripper sees opening
widths, a vacuum sees its [0, 1] binary range, etc.

Open / closed convention is purely policy-side here. This policy
treats "open" as the upper-limit end of the EE range and "closed"
as the lower. That fits the canonical parallel-gripper case (open =
max width = upper bound). It is reversed of natural meaning for a
vacuum (where lower=0=off=released and upper=1=on=engaged), and may
not produce a meaningful pose at all for a multi-DOF EE like a hand.
By design: the model interface stays clean; gripper-style framing is
this policy's problem alone.

The arm side of the emitted Action is a zero-velocity JointCommand
sized to num_arm_dof, so the policy is safe to drop into a diagram
without a separate arm-driving policy.

Stateful: latches the start time on the first step and times each
phase from there. Re-instantiate to restart the cycle.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.custom_types import EEPositionsVector
from manor.common.definitions.action import Action
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.manipulators.manipulator_model import IManipulatorModel


@attr.frozen
class GripperOpenClosePolicyConfig(MetisPolicyConfigBase):
    """
    Config for GripperOpenClosePolicy.

    open_hold_seconds / close_hold_seconds are the durations the
    policy holds at each end of the cycle before transitioning.
    num_cycles is how many full open->close pairs to run before
    settling at the closed setpoint permanently. num_arm_dof is the
    width of the fall-through zero-velocity arm command.

    The actual EE positions emitted come from the manipulator model's
    EE-position limits at policy-construction time -- they are not
    configurable here. That keeps the policy generic across robots
    and avoids encoding URDF-specific numbers in the YAML.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.GRIPPER_OPEN_CLOSE

    open_hold_seconds: float = 1.0
    close_hold_seconds: float = 1.0
    num_cycles: int = 1
    num_arm_dof: int = 0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "GripperOpenClosePolicyConfig"))


@attr.define
class GripperOpenClosePolicy:
    """
    Cycle the end-effector between the upper and lower bounds of the
    model's EE-position limits, holding each for the configured
    duration, for num_cycles cycles. Convention: open = upper bound,
    closed = lower bound. See module docstring on why this convention
    is gripper-biased.
    """

    open_positions: EEPositionsVector
    closed_positions: EEPositionsVector
    open_hold_seconds: float
    close_hold_seconds: float
    num_cycles: int
    num_arm_dof: int
    _start_time_s: float | None = attr.field(default=None, init=False)

    @classmethod
    def from_config(
        cls,
        config: GripperOpenClosePolicyConfig,
        manipulator_model: IManipulatorModel,
    ) -> Self:
        lower, upper = manipulator_model.get_ee_position_limits()
        return cls(
            # Policy convention: open = upper limit, closed = lower limit.
            open_positions=upper,
            closed_positions=lower,
            open_hold_seconds=config.open_hold_seconds,
            close_hold_seconds=config.close_hold_seconds,
            num_cycles=config.num_cycles,
            num_arm_dof=config.num_arm_dof,
        )

    def _select_ee_positions(self, elapsed_s: float) -> EEPositionsVector:
        cycle_duration = self.open_hold_seconds + self.close_hold_seconds
        total_cycle_duration = cycle_duration * self.num_cycles
        # After the configured number of cycles the policy settles at the closed setpoint and stays
        # there.
        if elapsed_s >= total_cycle_duration:
            return self.closed_positions
        # Within a cycle, the first open_hold_seconds is "open"; the remainder is "closed".
        within_cycle = elapsed_s % cycle_duration
        if within_cycle < self.open_hold_seconds:
            return self.open_positions
        return self.closed_positions

    def step(self, observation: Observation) -> Action:
        del observation
        header = TimestampHeader.from_system_time()
        # System-time clock is monotonic enough for the duty-cycle this policy targets; using
        # observation.header.system_ns would tie the policy to whatever system the observation came
        # from (helpful in replay; unhelpful here).
        now_s = header.system_ns * 1e-9
        if self._start_time_s is None:
            self._start_time_s = now_s
        elapsed = now_s - self._start_time_s

        ee_positions = self._select_ee_positions(elapsed)
        return Action(
            header=header,
            joint_command=JointCommand(
                header=header,
                joint_velocities=JointVelocities(
                    header=header,
                    velocities=np.zeros(self.num_arm_dof, dtype=np.float64),
                ),
            ),
            ee_command=EECommand(
                header=header,
                ee_positions=EEPositions(header=header, positions=ee_positions.copy()),
            ),
        )
