"""
GripperOpenClosePolicy: cycles a parallel-gripper opening width between
open and closed values, holding each for a configured duration. Repeats
for num_cycles cycles, then settles at the close width.

The arm side of the emitted Action is a zero-velocity JointCommand sized
to num_arm_dof, so the policy is safe to drop into a diagram without a
separate arm-driving policy.

The policy is stateful: it latches the start time on the first step and
times each open / close phase from there. Re-instantiate the policy to
start the cycle over.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.action import Action
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class GripperOpenClosePolicyConfig(MetisPolicyConfigBase):
    """
    Config for GripperOpenClosePolicy.

    open_width / close_width are EE-level opening widths (a single
    scalar per command per the EE interface; for the Lite6 parallel
    gripper, total opening across the two fingers).

    open_hold_seconds / close_hold_seconds are the durations the policy
    holds at each width before transitioning.

    num_cycles is how many full open->close pairs to run before settling
    at close_width permanently. num_arm_dof is the width of the
    fall-through zero-velocity arm command.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.GRIPPER_OPEN_CLOSE

    open_width: float = 0.04
    close_width: float = 0.0
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
    Cycle the gripper between open_width and close_width, holding each
    for the configured duration, for num_cycles cycles.
    """

    open_width: float
    close_width: float
    open_hold_seconds: float
    close_hold_seconds: float
    num_cycles: int
    num_arm_dof: int
    _start_time_s: float | None = attr.field(default=None, init=False)

    @classmethod
    def from_config(cls, config: GripperOpenClosePolicyConfig) -> Self:
        return cls(
            open_width=config.open_width,
            close_width=config.close_width,
            open_hold_seconds=config.open_hold_seconds,
            close_hold_seconds=config.close_hold_seconds,
            num_cycles=config.num_cycles,
            num_arm_dof=config.num_arm_dof,
        )

    def step(self, observation: Observation) -> Action:
        header = TimestampHeader.from_system_time()
        # System-time clock is monotonic enough for the duty-cycle this
        # policy targets; using observation.header.system_ns would tie
        # the policy to whatever system the observation came from
        # (helpful in replay; unhelpful here).
        now_s = header.system_ns * 1e-9
        if self._start_time_s is None:
            self._start_time_s = now_s
        elapsed = now_s - self._start_time_s

        width = self._select_width(elapsed)
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
                ee_positions=EEPositions(header=header, positions=np.array([width], dtype=np.float64)),
            ),
        )

    def _select_width(self, elapsed_s: float) -> float:
        cycle_duration = self.open_hold_seconds + self.close_hold_seconds
        total_cycle_duration = cycle_duration * self.num_cycles
        # After the configured number of cycles the policy settles at
        # close_width and stays there.
        if elapsed_s >= total_cycle_duration:
            return self.close_width
        # Within a cycle, the first open_hold_seconds is "open"; the
        # remainder is "close".
        within_cycle = elapsed_s % cycle_duration
        if within_cycle < self.open_hold_seconds:
            return self.open_width
        return self.close_width
