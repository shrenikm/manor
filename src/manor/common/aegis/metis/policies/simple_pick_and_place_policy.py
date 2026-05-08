"""
SimplePickAndPlacePolicy: walks the end-effector through a sequence of cartesian-pose waypoints
to pick a block from a known location and place it on a stack at another known location.

The policy is a translation of the previous deprecated-pliant simple_pick_and_place flow into a
Metis policy. It used to plan a PiecewisePose trajectory and emit cartesian twists; the new
version emits cartesian poses (translation only -- orientation is fixed downward-facing) so the
IKPassthroughController can position-IK to each waypoint and the driver's rate limiter handles
speed shaping. Gripper open/close transitions ride on the EECommand side of the same Action.

Waypoint sequence:

  1. above pick          (gripper OPEN)
  2. at pick             (gripper OPEN)   -- approach down
  3. at pick + hold      (gripper CLOSED) -- close on the block
  4. above pick          (gripper CLOSED) -- lift
  5. above place         (gripper CLOSED) -- traverse
  6. at place            (gripper CLOSED) -- approach down
  7. at place + hold     (gripper OPEN)   -- release
  8. above place         (gripper OPEN)   -- lift
  9. end pose            (gripper OPEN)   -- park

Each waypoint has a hold time -- duration the policy stays on it before advancing. Once the
last waypoint's hold elapses the policy keeps emitting the end pose (a "pose hold") so the arm
parks.

Stateful via the start-time latch. Assumes the IK plant configured downstream can reach all
configured waypoints; out-of-reach waypoints are the operator's problem.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml, require_number
from manor.common.custom_types import EEPositionsVector, NpVector3f64
from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError
from manor.manipulators.manipulator_model import IManipulatorModel

# Default downward-facing orientation: rotated 180 deg about world x so the EE z-axis points
# down. Matches the canonical pick orientation the deprecated pick-and-place hardcoded.
_DOWNWARD_QUATERNION_WXYZ = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)


def _parse_xyz(value: object, context: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        raise AegisConfigError(f"{context} must be a list of 3 numbers")
    return np.array([require_number(item, f"{context}[{i}]") for i, item in enumerate(value)], dtype=np.float64)


@attr.frozen
class SimplePickAndPlacePolicyConfig(MetisPolicyConfigBase):
    """
    Config for SimplePickAndPlacePolicy.

    pick_position is the (x, y, z) world-frame translation at which the EE grasps the block;
    place_position is where it deposits. end_position is the parking pose the EE settles at after
    the place. approach_height is how far above pick / place the EE waits before descending
    (0.05 m by default -- enough clearance over a 1-inch cube).

    open_width is the gripper opening width (m) for the OPEN waypoints; closed_width is the
    width when grasping. Both are emitted on the EECommand side; for vacuum-style EEs those map
    onto the [0, 1] on/off range via IManipulatorModel.

    move_duration_seconds is the time the policy holds at each non-grip waypoint to let the IK
    + rate limiter walk the arm there; gripper_settle_seconds is the additional dwell at the
    grip / release waypoints to let the gripper actuate before moving away. Both are dwell-only
    (the policy is not gating on pose convergence) so the operator should size them against the
    driver's joint speed limit and the cartesian distances configured here.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.SIMPLE_PICK_AND_PLACE

    pick_position: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    place_position: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    end_position: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    approach_height: float = 0.05
    open_width: float = 0.008
    closed_width: float = 0.0
    move_duration_seconds: float = 3.0
    gripper_settle_seconds: float = 1.0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(
            **parse_attrs_yaml(
                cls,
                d,
                "SimplePickAndPlacePolicyConfig",
                custom_parsers={
                    "pick_position": _parse_xyz,
                    "place_position": _parse_xyz,
                    "end_position": _parse_xyz,
                },
            )
        )


@attr.frozen
class _Waypoint:
    """
    One step of the pick-and-place sequence. translation is the EE target in world frame;
    open_gripper picks the OPEN vs CLOSED EE setpoint (resolved against the model's EE limits);
    duration_seconds is how long the policy dwells on this waypoint before advancing.
    """

    translation: NpVector3f64 = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    open_gripper: bool
    duration_seconds: float


@attr.define
class SimplePickAndPlacePolicy:
    """
    Time-driven cartesian waypoint walker. Builds a fixed sequence at construction, then on each
    step picks the active waypoint based on elapsed system time and emits its (cartesian pose,
    gripper EE positions) pair. Once past the final waypoint the policy parks at the end pose.
    """

    waypoints: tuple[_Waypoint, ...]
    open_ee_positions: EEPositionsVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    closed_ee_positions: EEPositionsVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    _start_time_s: float | None = attr.field(default=None, init=False)

    @classmethod
    def from_config(
        cls,
        config: SimplePickAndPlacePolicyConfig,
        manipulator_model: IManipulatorModel,
    ) -> Self:
        # Derive the open / closed EE setpoints against the manipulator model so the policy is
        # generic over EE shapes (parallel-gripper width vs vacuum on/off). The configured
        # open_width / closed_width are the EE-level scalars (gripper opening in metres for the
        # parallel case, [0, 1] for vacuum); we materialise them as length-1 EEPositions vectors
        # sized to the model's get_num_ee_dofs().
        num_ee_dofs = manipulator_model.get_num_ee_dofs()
        open_ee_positions = np.full(num_ee_dofs, config.open_width, dtype=np.float64)
        closed_ee_positions = np.full(num_ee_dofs, config.closed_width, dtype=np.float64)
        waypoints = cls._build_waypoints(config)
        return cls(
            waypoints=waypoints,
            open_ee_positions=open_ee_positions,
            closed_ee_positions=closed_ee_positions,
        )

    @staticmethod
    def _build_waypoints(config: SimplePickAndPlacePolicyConfig) -> tuple[_Waypoint, ...]:
        above_pick = config.pick_position + np.array([0.0, 0.0, config.approach_height], dtype=np.float64)
        above_place = config.place_position + np.array([0.0, 0.0, config.approach_height], dtype=np.float64)
        move = config.move_duration_seconds
        settle = config.gripper_settle_seconds
        # The dwell at grip / release combines move-down and gripper-actuation time; the gripper
        # transitions in the middle of that dwell as the policy advances to the next waypoint.
        return (
            _Waypoint(translation=above_pick, open_gripper=True, duration_seconds=move),
            _Waypoint(translation=config.pick_position, open_gripper=True, duration_seconds=move),
            _Waypoint(translation=config.pick_position, open_gripper=False, duration_seconds=settle),
            _Waypoint(translation=above_pick, open_gripper=False, duration_seconds=move),
            _Waypoint(translation=above_place, open_gripper=False, duration_seconds=move),
            _Waypoint(translation=config.place_position, open_gripper=False, duration_seconds=move),
            _Waypoint(translation=config.place_position, open_gripper=True, duration_seconds=settle),
            _Waypoint(translation=above_place, open_gripper=True, duration_seconds=move),
            _Waypoint(translation=config.end_position, open_gripper=True, duration_seconds=move),
        )

    def step(self, observation: Observation) -> Action:
        del observation
        header = TimestampHeader.from_system_time()
        now_s = header.system_ns * 1e-9
        if self._start_time_s is None:
            self._start_time_s = now_s
        elapsed = now_s - self._start_time_s
        waypoint = self._select_waypoint(elapsed)
        return self._make_action(header, waypoint)

    def _select_waypoint(self, elapsed_s: float) -> _Waypoint:
        # Walk the cumulative dwell timeline. After the last waypoint's window the policy parks
        # at the final waypoint forever -- no error / wraparound, just a pose hold.
        cumulative = 0.0
        for wp in self.waypoints:
            cumulative += wp.duration_seconds
            if elapsed_s < cumulative:
                return wp
        return self.waypoints[-1]

    def _make_action(self, header: TimestampHeader, waypoint: _Waypoint) -> Action:
        ee_positions = self.open_ee_positions if waypoint.open_gripper else self.closed_ee_positions
        return Action(
            header=header,
            cartesian_command=CartesianCommand(
                header=header,
                cartesian_pose=CartesianPose(
                    header=header,
                    translation=np.asarray(waypoint.translation, dtype=np.float64).copy(),
                    orientation=_DOWNWARD_QUATERNION_WXYZ.copy(),
                ),
            ),
            ee_command=EECommand(
                header=header,
                ee_positions=EEPositions(header=header, positions=ee_positions.copy()),
            ),
        )
