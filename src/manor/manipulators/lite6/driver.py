"""
Lite6 hardware driver, backed by xarm-python-sdk.

Wraps ``XArmAPI`` to satisfy ``IManipulatorDriver``. The bring-up /
tear-down sequences (prime / unprime) live in
``manor.manipulators.lite6.xarm_helpers`` so this driver and the
standalone ``lite6_cli`` share a single validated implementation.

End-effector read/write semantics on the Lite6 are limited by the SDK:

* The SDK exposes joint-only state (the 6 arm positions / velocities);
  gripper position is not readable. Both ``read_ee_*`` methods return
  ``None``.
* Gripper actuation is binary (open / close / stop). ``write_ee_*``
  methods threshold the supplied EE vector to map onto those calls.

The driver does NOT disconnect on ``unprime`` -- motors stay energized
and the TCP session stays open so subsequent re-prime cycles (e.g. after
a stale-command watchdog trip in ``HardwareManipulatorBackend``) skip
the brake-release / encoder-relock latency. The session is released
only when the process exits.
"""

from __future__ import annotations

import contextlib
import io
from typing import Self, override

import attr
import numpy as np

from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.custom_types import JointPositionsVector, JointVelocitiesVector
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import Lite6DriverError
from manor.common.logging_utils import ManorLogger
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.lite6.xarm_helpers import (
    XArmCallError,
    XArmMode,
    XArmState,
    check_xarm_call,
)
from manor.manipulators.lite6.xarm_helpers import (
    prime as xarm_prime,
)
from manor.manipulators.lite6.xarm_helpers import (
    switch_mode as xarm_switch_mode,
)
from manor.manipulators.lite6.xarm_helpers import (
    unprime as xarm_unprime,
)
from manor.manipulators.manipulator_driver import IManipulatorDriver

# The xarm SDK prints ``SDK_VERSION: <ver>`` to stdout on import, with
# no off switch. Swallow it with a stdout redirect during the import so
# every aegis CLI / runner invocation isn't preceded by that banner.
# xarm is a hard dependency (pyproject.toml) -- if the import fails,
# that's a real environment problem and we want the ImportError to
# surface, not be swallowed into a stub None.
with contextlib.redirect_stdout(io.StringIO()):
    from xarm.wrapper import XArmAPI

# Default IP that the deprecated codebase used. Override per-instance.
_LITE6_DEFAULT_IP = "192.168.1.178"

# Heuristic threshold for collapsing a continuous EE position vector
# onto the parallel gripper's binary open / close command. The
# parallel-gripper URDFs use ~0.008 m for "open"; halfway is fine.
_LITE6_PARALLEL_GRIPPER_OPEN_THRESHOLD_M = 0.004


@attr.frozen
class Lite6DriverConfig:
    """
    Tunable knobs for the Lite6 hardware driver. Identity / connectivity (model, ip) live on the
    driver itself; this config covers behaviour the operator may want to dial per-policy without
    rebuilding the stack.

    joint_speed_limit_rad_s caps the per-joint speed enforced client-side in
    write_joint_positions. The xarm SDK accepts a speed= argument on set_servo_angle_j and
    forwards it to the firmware as mvvelo, but the controller treats it as reserved (ignored)
    for mode 1 streaming -- documented in the SDK and verified empirically (the arm sprints
    toward whatever target the streaming call sets, bounded only by the global joint_speed_limit
    upper bound near pi rad/s, which is enough to trip a 'Not Ready' fault on far-away IK
    solutions). The driver instead clamps each new commanded position so it advances by at most
    joint_speed_limit_rad_s * dt from the previously commanded position. Required (no default)
    -- every hardware run must declare it in the YAML so the operator has consciously chosen a
    value matched to the policy.
    """

    joint_speed_limit_rad_s: float = attr.field(validator=attr.validators.gt(0.0))

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "lite6_driver_config"))


@attr.define
class Lite6Driver(IManipulatorDriver):
    """
    IManipulatorDriver implementation that talks to a real Lite6 over
    the xarm SDK.
    """

    model: Lite6Model
    config: Lite6DriverConfig
    ip: str = _LITE6_DEFAULT_IP
    _arm: XArmAPI = attr.field(init=False)
    # Sticky-mode cache. Source of truth for the current operating mode -- the firmware's heartbeat-
    # cached arm.mode lags by ~200 ms and isn't reliable to compare against. None until prime() runs;
    # prime / unprime always end in mode POSITION (mode 0) so we set it to that. write_joint_positions
    # / write_joint_velocities lazily switch into SERVO_POSITION (mode 1) / VELOCITY (mode 4) on
    # first use and on every shape transition; characterised cost is ~190-210 ms per switch (mostly
    # the heartbeat-poll inside switch_mode), so per-tick switching is infeasible -- caching is what
    # makes mixed-shape policies tolerable.
    _current_mode: XArmMode | None = attr.field(init=False, default=None)
    # Client-side rate limiter state for write_joint_positions. The xarm SDK's per-call speed=
    # argument is ignored by the firmware for set_servo_angle_j (mode 1 streaming), so we clamp
    # the advancement of the commanded position ourselves. The previously commanded
    # (position, timestamp) pair lives inside a single JointPositions -- the header carries the
    # monotonic_ns we need for dt, the positions field carries the vector we step from. None
    # until the first write after prime / resume / unprime / a mode change.
    _last_commanded_joint_positions: JointPositions | None = attr.field(init=False, default=None)
    _logger: ManorLogger = attr.field(init=False)

    @_arm.default
    def _initialize_arm(self) -> XArmAPI:
        # do_not_open=True keeps the constructor cheap and side-effect free: no socket is opened until
        # prime() runs. Without this, instantiating the driver in a test (or in any code path that
        # imports the hardware backend without intending to actually drive an arm) would synchronously
        # try to reach the controller IP and fail. The first prime() call connects + brings the arm up.
        return XArmAPI(port=self.ip, is_radian=True, do_not_open=True)

    @_logger.default
    def _initialize_logger(self) -> ManorLogger:
        return ManorLogger(self.__class__.__name__)

    @override
    def get_num_dof(self) -> int:
        return self.model.get_num_dof()

    @override
    def get_num_ee_dofs(self) -> int:
        return self.model.get_num_ee_dofs()

    @override
    def prime(self) -> None:
        # Bring-up sequence shared with lite6_cli: clean errors, motion_enable, settle, mode 0 /
        # READY, soft-recover on latched servo errors, move to PRIME. Always leaves the arm in mode
        # POSITION (mode 0). The XArmAPI was constructed with do_not_open=True, so open the TCP
        # session before issuing SDK calls. arm.connect() is cheap if already open (we guard on
        # arm.connected anyway).
        try:
            if not self._arm.connected:
                self._arm.connect()
            xarm_prime(self._arm, log_fn=self._logger.info)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc
        self._current_mode = XArmMode.POSITION
        self._reset_position_limiter()

    @override
    def unprime(self) -> None:
        # Inverse of prime: switch back to mode 0, move to ZERO, set_state(STOP). Does NOT call
        # motion_enable(False), does NOT disconnect the TCP session, and does NOT null self._arm --
        # the watchdog / aegis-shutdown paths may re-prime on the same driver instance. xarm_unprime
        # always finishes by setting mode POSITION, so reset the cache to match.
        try:
            xarm_unprime(self._arm, log_fn=self._logger.info)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc
        self._current_mode = XArmMode.POSITION
        self._reset_position_limiter()

    def _ensure_mode(self, target: XArmMode, *, write_kind: str) -> None:
        # Sticky-mode gate for the streaming write paths. switch_mode goes through STOP -> set_mode
        # -> READY and waits for the heartbeat to confirm; expensive (~200 ms) and aborts any in-
        # flight motion, so we only call it on transitions. Mid-motion switches are safe (the
        # embedded set_state(STOP) decelerates cleanly, no faults), characterised on hardware on
        # 2026-05-04. _current_mode is None only before prime(); a write before prime is a usage
        # error -- raise rather than silently driving a not-ready arm. Any actual mode change
        # also resets the position limiter -- the previously commanded streaming target is stale
        # if motion went through a different mode in between (e.g. a VELOCITY phase moved the arm
        # away from the last commanded JointPositions), so we re-seed from the measured pose on
        # the next joint-position write. write_kind is the caller's label for the kind of command
        # that triggered the check (e.g. "joint_positions", "joint_velocities") -- used purely for
        # the mismatch log so a policy whose command shape disagrees with the previous tick's
        # shape shows up in the unified manor log stream.
        if self._current_mode is None:
            raise Lite6DriverError("write call issued before prime(); the arm has no operating mode set")
        if self._current_mode is target:
            return
        self._logger.info(
            f"command kind={write_kind} requires mode={target.name}; "
            f"driver was in mode={self._current_mode.name} -- switching"
        )
        try:
            xarm_switch_mode(self._arm, mode=target, log_fn=self._logger.info)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc
        self._current_mode = target
        self._reset_position_limiter()

    @override
    def halt(self) -> None:
        # Refuse motion at the controller level without moving or changing mode -- the arm holds
        # its current pose, motors stay energized, mode stays as set by prime. The watchdog calls
        # this when the action stream goes stale; resume() flips state back to READY.
        self._logger.info("halt: set_state STOP -- arm holds pose, motors energized, mode preserved")
        self._call(self._arm.set_state(state=XArmState.STOP), "set_state(stop)")

    @override
    def resume(self) -> None:
        # Inverse of halt: re-arm the controller for motion. set_state(READY) is idempotent on the
        # firmware side when state is already READY, so spurious resume calls are harmless. Also
        # reset the position limiter -- if the arm decelerated under halt the firmware's actual
        # pose may have drifted away from the last commanded JointPositions, and re-seeding from
        # measured on the next write avoids a snap when streaming resumes.
        self._logger.info("resume: set_state READY -- position limiter will re-seed from measured on next write")
        self._call(self._arm.set_state(state=XArmState.READY), "set_state(ready)")
        self._reset_position_limiter()

    @override
    def read_joint_positions(self) -> JointPositions:
        positions, _ = self._read_joint_state()
        return JointPositions(
            header=TimestampHeader.from_system_time(),
            positions=positions,
        )

    @override
    def read_joint_velocities(self) -> JointVelocities:
        _, velocities = self._read_joint_state()
        return JointVelocities(
            header=TimestampHeader.from_system_time(),
            velocities=velocities,
        )

    @override
    def read_ee_positions(self) -> EEPositions | None:
        # The xarm SDK does not expose gripper position as readable state.
        return None

    @override
    def read_ee_velocities(self) -> EEVelocities | None:
        # Same as read_ee_positions: not exposed by the xarm SDK.
        return None

    @override
    def write_joint_positions(self, joint_positions: JointPositions) -> None:
        # set_servo_angle_j requires mode SERVO_POSITION (mode 1). Switch lazily on the first
        # joint-position write after prime / a velocity write; subsequent same-shape writes are a
        # cheap cache hit. The commanded angles are clamped client-side via
        # _compute_rate_limited_target so each call advances by at most
        # joint_speed_limit_rad_s * dt from the previously commanded position; the SDK's own
        # speed= argument is reserved/ignored by the firmware for this call so we don't pass it.
        self._ensure_mode(XArmMode.SERVO_POSITION, write_kind="joint_positions")
        target = joint_positions.positions.astype(np.float64)
        commanded = self._compute_rate_limited_target(target)
        self._call(
            self._arm.set_servo_angle_j(
                angles=commanded.tolist(),
                is_radian=True,
            ),
            "set_servo_angle_j",
        )

    def _compute_rate_limited_target(self, target: JointPositionsVector) -> JointPositionsVector:
        # On the first call after prime / unprime / resume / a mode change the limiter has no
        # history, so we seed it from the arm's measured pose and emit that as the commanded
        # value. The first SDK call is then a no-op write (commanded == measured) which aligns
        # the streaming target with where the arm actually is; subsequent calls advance toward
        # the policy target at the configured speed.
        now = self._now_timestamp_header()
        if self._last_commanded_joint_positions is None:
            measured, _ = self._read_joint_state()
            self._last_commanded_joint_positions = JointPositions(header=now, positions=measured)
            return measured
        last = self._last_commanded_joint_positions
        dt_s = (now.monotonic_ns - last.header.monotonic_ns) / 1e9
        max_step = self.config.joint_speed_limit_rad_s * dt_s
        delta = target - last.positions
        commanded = last.positions + np.clip(delta, -max_step, max_step)
        self._last_commanded_joint_positions = JointPositions(header=now, positions=commanded)
        return commanded

    def _now_timestamp_header(self) -> TimestampHeader:
        # Indirection point so unit tests can patch the limiter's clock per-instance without
        # mutating the global time module (which would break TimestampHeader.from_system_time
        # and every other monotonic_ns reader running in the same process).
        return TimestampHeader.from_system_time()

    def _reset_position_limiter(self) -> None:
        self._last_commanded_joint_positions = None

    @override
    def write_joint_velocities(self, joint_velocities: JointVelocities) -> None:
        # vc_set_joint_velocity requires mode VELOCITY (mode 4). Same sticky-cache pattern as
        # write_joint_positions; the arm's controller silently no-ops vc_set_joint_velocity if the
        # mode is wrong, which is exactly the bug that motivated this whole refactor.
        self._ensure_mode(XArmMode.VELOCITY, write_kind="joint_velocities")
        self._call(
            self._arm.vc_set_joint_velocity(
                speeds=joint_velocities.velocities.astype(np.float64).tolist(),
                is_radian=True,
                duration=0,
            ),
            "vc_set_joint_velocity",
        )

    @override
    def write_ee_positions(self, ee_positions: EEPositions) -> None:
        # Map the continuous EE position vector onto the SDK's binary
        # open / close command via a threshold on the max element.
        is_open = bool(np.any(np.abs(ee_positions.positions) > _LITE6_PARALLEL_GRIPPER_OPEN_THRESHOLD_M))
        self._send_gripper_command(open_command=is_open)

    @override
    def write_ee_velocities(self, ee_velocities: EEVelocities) -> None:
        # Sign of the velocity vector picks the direction; zero stops.
        max_abs = float(np.max(np.abs(ee_velocities.velocities))) if ee_velocities.velocities.size > 0 else 0.0
        if max_abs == 0.0:
            self._send_gripper_stop()
        else:
            is_open = bool(np.max(ee_velocities.velocities) > 0.0)
            self._send_gripper_command(open_command=is_open)

    def _read_joint_state(self) -> tuple[JointPositionsVector, JointVelocitiesVector]:
        ret_code, raw = self._arm.get_joint_states(is_radian=True)
        try:
            check_xarm_call(ret_code, "get_joint_states", arm=self._arm)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc
        # The SDK returns 7-element vectors regardless of arm DOF; the
        # 7th value is auxiliary (used on 7-DOF models). Slice to the
        # arm's actual DOF count.
        positions_raw, velocities_raw, _ = raw
        positions = np.asarray(positions_raw, dtype=np.float64)[:LITE6_ARM_DOF]
        velocities = np.asarray(velocities_raw, dtype=np.float64)[:LITE6_ARM_DOF]
        return positions, velocities

    def _send_gripper_command(self, open_command: bool) -> None:
        if self.model.variant in (Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE):
            ret_code = self._arm.open_lite6_gripper() if open_command else self._arm.close_lite6_gripper()
            self._call(ret_code, "open_lite6_gripper" if open_command else "close_lite6_gripper")
        elif self.model.variant is Lite6Variant.VACUUM_GRIPPER:
            self._call(self._arm.set_vacuum_gripper(on=open_command), "set_vacuum_gripper")
        else:
            raise Lite6DriverError(f"Unhandled Lite6 variant: {self.model.variant!r}")

    def _send_gripper_stop(self) -> None:
        if self.model.variant in (Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE):
            self._call(self._arm.stop_lite6_gripper(), "stop_lite6_gripper")
        elif self.model.variant is Lite6Variant.VACUUM_GRIPPER:
            # Stopping a vacuum gripper just means turning it off.
            self._call(self._arm.set_vacuum_gripper(on=False), "set_vacuum_gripper")
        else:
            raise Lite6DriverError(f"Unhandled Lite6 variant: {self.model.variant!r}")

    def _call(self, ret_code: int | tuple, op: str) -> None:
        # Thin adapter around the shared check_xarm_call so driver
        # callers raise Lite6DriverError (the project-specific
        # exception) rather than the generic XArmCallError the helpers
        # speak in.
        try:
            check_xarm_call(ret_code, op, arm=self._arm)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc
