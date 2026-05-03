"""
Lite6 hardware driver, backed by xarm-python-sdk.

Wraps ``XArmAPI`` to satisfy ``IManipulatorDriver``. End-effector
read/write semantics on the Lite6 are limited by the SDK:

* The SDK exposes joint-only state (the 6 arm positions / velocities);
  gripper position is not readable. Both ``read_ee_*`` methods return
  ``None``.
* Gripper actuation is binary (open / close / stop). ``write_ee_*``
  methods threshold the supplied EE vector to map onto those calls.
"""

from __future__ import annotations

import contextlib
import io
from typing import Any, override

import attr
import numpy as np

from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import Lite6DriverError
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_driver import IManipulatorDriver

# The xarm SDK prints ``SDK_VERSION: <ver>`` to stdout on import, with
# no off switch. Swallow it with a stdout redirect during the import so
# every aegis CLI / runner invocation isn't preceded by that banner.
# xarm is now a hard dependency (pyproject.toml) -- if the import
# fails, that's a real environment problem and we want the
# ``ImportError`` to surface, not be swallowed into a stub ``None``.
with contextlib.redirect_stdout(io.StringIO()):
    from xarm.wrapper import XArmAPI

# Default IP that the deprecated codebase used. Override per-instance.
_LITE6_DEFAULT_IP = "192.168.1.178"

# xarm mode constants.
_XARM_MODE_SERVO_POSITION = 1
_XARM_MODE_VELOCITY = 4

# xarm state constants.
_XARM_STATE_READY = 0
_XARM_STATE_STOP = 4

# Heuristic threshold for collapsing a continuous EE position vector
# onto the parallel gripper's binary open / close command. The
# parallel-gripper URDFs use ~0.008 m for "open"; halfway is fine.
_LITE6_PARALLEL_GRIPPER_OPEN_THRESHOLD_M = 0.004


@attr.define
class Lite6Driver(IManipulatorDriver):
    """
    ``IManipulatorDriver`` implementation that talks to a real Lite6 over
    the xarm SDK.
    """

    model: Lite6Model
    ip: str = _LITE6_DEFAULT_IP
    _arm: XArmAPI = attr.field(init=False)

    @_arm.default
    def _initialize_arm(self) -> XArmAPI:
        return XArmAPI(port=self.ip, is_radian=True)

    @override
    def get_num_dof(self) -> int:
        return self.model.get_num_dof()

    @override
    def get_num_ee_dofs(self) -> int:
        return self.model.get_num_ee_dofs()

    @override
    def prime(self) -> None:
        self._check(self._arm.clean_error(), "clean_error")
        self._check(self._arm.motion_enable(enable=True), "motion_enable")
        self._check(self._arm.set_mode(mode=_XARM_MODE_SERVO_POSITION), "set_mode")
        self._check(self._arm.set_state(state=_XARM_STATE_READY), "set_state")

    @override
    def unprime(self) -> None:
        if self._arm is None:
            return
        try:
            self._arm.set_state(state=_XARM_STATE_STOP)
            self._arm.motion_enable(enable=False)
        finally:
            self._arm = None

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
        self._check(
            self._arm.set_servo_angle_j(
                angles=joint_positions.positions.astype(np.float64).tolist(),
                is_radian=True,
            ),
            "set_servo_angle_j",
        )

    @override
    def write_joint_velocities(self, joint_velocities: JointVelocities) -> None:
        self._check(
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
        if ret_code != 0:
            raise Lite6DriverError(f"get_joint_states failed (code={ret_code})")
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
            self._check(ret_code, "open_lite6_gripper" if open_command else "close_lite6_gripper")
        elif self.model.variant is Lite6Variant.VACUUM_GRIPPER:
            self._check(self._arm.set_vacuum_gripper(on=open_command), "set_vacuum_gripper")
        else:
            raise Lite6DriverError(f"Unhandled Lite6 variant: {self.model.variant!r}")

    def _send_gripper_stop(self) -> None:
        if self.model.variant in (Lite6Variant.PARALLEL_GRIPPER_NORMAL, Lite6Variant.PARALLEL_GRIPPER_REVERSE):
            self._check(self._arm.stop_lite6_gripper(), "stop_lite6_gripper")
        elif self.model.variant is Lite6Variant.VACUUM_GRIPPER:
            # Stopping a vacuum gripper just means turning it off.
            self._check(self._arm.set_vacuum_gripper(on=False), "set_vacuum_gripper")
        else:
            raise Lite6DriverError(f"Unhandled Lite6 variant: {self.model.variant!r}")

    def _check(self, ret_code: int | tuple, op: str) -> None:
        # Some xarm calls return a plain int; others return a tuple
        # whose first element is the status code.
        code = ret_code[0] if isinstance(ret_code, tuple) else ret_code
        if code != 0:
            try:
                self._arm.emergency_stop()
            except Exception:
                pass
            raise Lite6DriverError(f"xarm SDK call {op!r} failed (code={code})")
