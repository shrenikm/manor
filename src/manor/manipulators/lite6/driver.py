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
from typing import override

import attr
import numpy as np

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

# Operating mode the driver primes into. Mode 1 (servo position) is the
# low-latency joint-streaming mode write_joint_positions targets;
# write_joint_velocities flips to mode 4 on demand inside the call.
_DRIVER_PRIME_MODE = XArmMode.SERVO_POSITION


@attr.define
class Lite6Driver(IManipulatorDriver):
    """
    IManipulatorDriver implementation that talks to a real Lite6 over
    the xarm SDK.
    """

    model: Lite6Model
    ip: str = _LITE6_DEFAULT_IP
    _arm: XArmAPI = attr.field(init=False)
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
        # Bring-up sequence shared with lite6_cli: clean errors,
        # motion_enable, settle, mode 0 / READY, soft-recover on
        # latched servo errors, move to PRIME, then switch into the
        # streaming mode the production write paths target. The
        # XArmAPI was constructed with do_not_open=True, so open the
        # TCP session before issuing SDK calls. arm.connect() is
        # idempotent on the SDK side -- calling it again on an
        # already-open session is a cheap no-op via arm.connected.
        try:
            if not self._arm.connected:
                self._arm.connect()
            xarm_prime(self._arm, mode=_DRIVER_PRIME_MODE, log_fn=self._logger.info)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc

    @override
    def unprime(self) -> None:
        # Inverse of prime: switch back to mode 0, move to ZERO,
        # set_state(STOP). Does NOT call motion_enable(False), does NOT
        # disconnect the TCP session, and does NOT null self._arm --
        # the watchdog / aegis-shutdown paths may re-prime on the same
        # driver instance.
        try:
            xarm_unprime(self._arm, log_fn=self._logger.info)
        except XArmCallError as exc:
            raise Lite6DriverError(str(exc)) from exc

    @override
    def halt(self) -> None:
        # Refuse motion at the controller level without moving or changing mode -- the arm holds
        # its current pose, motors stay energized, mode stays as set by prime. The watchdog calls
        # this when the action stream goes stale; resume() flips state back to READY.
        self._call(self._arm.set_state(state=XArmState.STOP), "set_state(stop)")

    @override
    def resume(self) -> None:
        # Inverse of halt: re-arm the controller for motion. set_state(READY) is idempotent on the
        # firmware side when state is already READY, so spurious resume calls are harmless.
        self._call(self._arm.set_state(state=XArmState.READY), "set_state(ready)")

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
        self._call(
            self._arm.set_servo_angle_j(
                angles=joint_positions.positions.astype(np.float64).tolist(),
                is_radian=True,
            ),
            "set_servo_angle_j",
        )

    @override
    def write_joint_velocities(self, joint_velocities: JointVelocities) -> None:
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

    def _read_joint_state(self) -> tuple[np.ndarray, np.ndarray]:
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
