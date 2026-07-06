"""
reBot B601 DM hardware driver, backed by the motorbridge SDK.

Wraps a RebotB601DmBus (see motorbridge_helpers) to satisfy IManipulatorDriver. The control strategy follows
the vendor stack (reBotArm_control_py / ROS2 controller / official LeRobot integration):

* Arm joint positions stream in POS_VEL mode: the DM firmware chases each target under a per-joint velocity
limit, and the driver additionally clamps how far the commanded target may advance per tick
(joint_speed_limit_rad_s), mirroring the lite6 driver's client-side rate limiter.
* Arm joint velocities stream in VEL mode, with the same sticky mode cache as the lite6 driver (mode
transitions are bus-expensive; per-tick switching is infeasible).
* The gripper is ONLY ever driven in FORCE_POS mode: firmware position control with a hard torque ceiling
(gripper_torque_ratio, a fraction of max motor torque). Plain position control applies full motor torque on
contact and has physically broken the 3D printed gripper linkage -- every gripper write in this driver goes
through the torque-capped path, and the config validator refuses ratios above a safe ceiling.

Unlike the lite6, the SDK exposes gripper position / velocity feedback, so read_ee_* return real state (the
jaw width and its rate) rather than None.

unprime moves the arm to REST (the vendor home pose, where the folded arm is safe to de-energize) and
disables the motors -- this is the vendor's own shutdown behaviour. The serial bridge stays open so a
subsequent prime on the same driver instance skips the reconnect.
"""

from __future__ import annotations

from typing import Self, override

import attr
import numpy as np
from motorbridge import Mode

from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.custom_types import JointPositionsVector
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import RebotB601DmDriverError
from manor.common.logging_utils import ManorLogger
from manor.manipulators.manipulator_driver import IManipulatorDriver
from manor.manipulators.rebot_b601_dm.model import RebotB601DmModel
from manor.manipulators.rebot_b601_dm.motorbridge_helpers import (
    REBOT_B601_DM_DEFAULT_CHANNEL,
    REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX,
    MotorBridgeCallError,
    RebotB601DmBus,
    gripper_motor_rad_s_to_width_m_s,
    gripper_motor_rad_to_width,
    gripper_width_to_motor_rad,
)
from manor.manipulators.rebot_b601_dm.motorbridge_helpers import (
    prime as bus_prime,
)
from manor.manipulators.rebot_b601_dm.motorbridge_helpers import (
    unprime as bus_unprime,
)


@attr.frozen
class RebotB601DmDriverConfig:
    """
    Tunable knobs for the reBot B601 DM hardware driver. Identity / connectivity (the serial channel) lives
    on the driver itself; this config covers behaviour the operator may want to dial per-policy without
    rebuilding the stack. Both fields are required (no defaults) so every hardware run consciously declares
    them in the YAML.

    joint_speed_limit_rad_s caps the per-joint speed enforced client-side in write_joint_positions: each new
    commanded position advances by at most joint_speed_limit_rad_s * dt from the previously commanded
    position. The firmware's own POS_VEL velocity limit provides a second bound, but the client-side clamp
    is what keeps a far-away streamed target (e.g. a bad IK solution) from turning into a lunge.

    gripper_torque_ratio is the FORCE_POS torque ceiling for every gripper command, as a fraction of the
    gripper motor's maximum torque. THE GRIPPER LINKAGE IS 3D PRINTED AND FULL MOTOR TORQUE BREAKS IT; the
    vendor's LeRobot integration grips at 0.07 and that holds objects fine. The validator refuses anything
    above REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX (0.2).
    """

    joint_speed_limit_rad_s: float = attr.field(validator=attr.validators.gt(0.0))
    gripper_torque_ratio: float = attr.field(
        validator=[
            attr.validators.gt(0.0),
            attr.validators.le(REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX),
        ]
    )

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "rebot_b601_dm_driver_config"))


@attr.define
class RebotB601DmDriver(IManipulatorDriver):
    """
    IManipulatorDriver implementation that talks to a real reBot B601 DM over motorbridge.
    """

    model: RebotB601DmModel
    config: RebotB601DmDriverConfig
    channel: str = REBOT_B601_DM_DEFAULT_CHANNEL
    _bus: RebotB601DmBus = attr.field(init=False)
    # Sticky arm-mode cache. POS_VEL / VEL transitions rewrite motor registers over the serial bridge
    # (~0.05 s per motor plus a group settle), so we only switch on shape transitions, mirroring the lite6
    # driver. None until prime() runs; prime always ends in POS_VEL.
    _current_arm_mode: Mode | None = attr.field(init=False, default=None)
    # Halted flag. halt() freezes the arm at its current pose and refuses subsequent writes until resume().
    # The DM motors have no firmware STOP state equivalent to the xarm's, so the refusal is enforced here at
    # the driver layer (the aegis hardware backend also gates sends while halted; this is defence in depth).
    _halted: bool = attr.field(init=False, default=False)
    # Client-side rate limiter state for write_joint_positions; same design as the lite6 driver. The header
    # carries the monotonic_ns for dt, positions carry the vector we step from. None until the first write
    # after prime / resume / a mode change, at which point it re-seeds from the measured pose.
    _last_commanded_joint_positions: JointPositions | None = attr.field(init=False, default=None)
    _logger: ManorLogger = attr.field(init=False)

    @_bus.default
    def _initialize_bus(self) -> RebotB601DmBus:
        # Construction is cheap and side-effect free; no serial port is opened until prime() runs.
        return RebotB601DmBus(channel=self.channel)

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
        # Bring-up sequence shared with rebot_b601_dm_cli: open the bus, clear errors, enable, configure
        # modes (arm POS_VEL, gripper FORCE_POS), close the gripper softly, move to PRIME.
        try:
            bus_prime(
                self._bus,
                gripper_torque_ratio=self.config.gripper_torque_ratio,
                log_fn=self._logger.info,
            )
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc
        self._current_arm_mode = Mode.POS_VEL
        self._halted = False
        self._reset_position_limiter()

    @override
    def unprime(self) -> None:
        # Inverse of prime: close the gripper, park at REST, disable the motors. The serial bridge stays
        # open so a subsequent prime skips the reconnect. A watchdog halt must not block parking, so clear
        # the halted flag first.
        self._halted = False
        try:
            if self._current_arm_mode is not Mode.POS_VEL:
                self._bus.set_arm_mode(Mode.POS_VEL, log_fn=self._logger.info)
            bus_unprime(
                self._bus,
                gripper_torque_ratio=self.config.gripper_torque_ratio,
                log_fn=self._logger.info,
            )
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc
        self._current_arm_mode = None
        self._reset_position_limiter()

    def _ensure_arm_mode(self, target: Mode) -> None:
        # Sticky-mode gate for the streaming write paths, mirroring the lite6 driver. A write before prime
        # is a usage error -- raise rather than silently driving a not-ready arm. Any actual mode change
        # resets the position limiter so the next joint-position write re-seeds from the measured pose.
        if self._current_arm_mode is None:
            raise RebotB601DmDriverError("write call issued before prime(); the arm has no operating mode set")
        if self._current_arm_mode is target:
            return
        try:
            self._bus.set_arm_mode(target, log_fn=self._logger.info)
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc
        self._current_arm_mode = target
        self._reset_position_limiter()

    @override
    def halt(self) -> None:
        # Freeze motion at the current pose: latch the measured positions as the streaming target (POS_VEL
        # holds a latched target indefinitely) and refuse subsequent writes until resume(). Motors stay
        # energized and the operating mode is preserved. If the arm was in VEL mode, zero the velocity
        # target instead -- re-latching a position in VEL mode would be a mode switch, which halt must not
        # do.
        try:
            if self._current_arm_mode is Mode.VEL:
                self._bus.send_arm_vel(np.zeros(self.get_num_dof(), dtype=np.float64))
            elif self._current_arm_mode is Mode.POS_VEL:
                positions, _ = self._bus.read_arm_state()
                self._bus.send_arm_pos_vel(positions)
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc
        self._halted = True

    @override
    def resume(self) -> None:
        # Undo a prior halt. Idempotent -- safe to call when not halted. The position limiter re-seeds from
        # the measured pose on the next write, so streaming resumes without a snap.
        self._halted = False
        self._reset_position_limiter()

    @override
    def read_joint_positions(self) -> JointPositions:
        positions, _ = self._read_arm_state()
        return JointPositions(
            header=TimestampHeader.from_system_time(),
            positions=positions,
        )

    @override
    def read_joint_velocities(self) -> JointVelocities:
        _, velocities = self._read_arm_state()
        return JointVelocities(
            header=TimestampHeader.from_system_time(),
            velocities=velocities,
        )

    @override
    def read_ee_positions(self) -> EEPositions | None:
        # The gripper motor reports position; map motor radians onto the physical jaw width so the EE
        # vector matches the model's EE convention.
        motor_rad, _ = self._read_gripper_state()
        return EEPositions(
            header=TimestampHeader.from_system_time(),
            positions=np.array([gripper_motor_rad_to_width(motor_rad)], dtype=np.float64),
        )

    @override
    def read_ee_velocities(self) -> EEVelocities | None:
        _, motor_rad_s = self._read_gripper_state()
        return EEVelocities(
            header=TimestampHeader.from_system_time(),
            velocities=np.array([gripper_motor_rad_s_to_width_m_s(motor_rad_s)], dtype=np.float64),
        )

    def _read_arm_state(self) -> tuple[np.ndarray, np.ndarray]:
        try:
            return self._bus.read_arm_state()
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc

    def _read_gripper_state(self) -> tuple[float, float]:
        try:
            return self._bus.read_gripper_state()
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc

    def _now_timestamp_header(self) -> TimestampHeader:
        # Indirection point so unit tests can patch the limiter's clock per-instance without mutating the
        # global time module.
        return TimestampHeader.from_system_time()

    def _reset_position_limiter(self) -> None:
        self._last_commanded_joint_positions = None

    def _compute_rate_limited_target(self, target: JointPositionsVector) -> JointPositionsVector:
        # Same design as the lite6 driver: on the first call after prime / resume / a mode change the
        # limiter has no history, so seed from the measured pose and emit that (a no-op write that aligns
        # the streaming target with where the arm actually is); subsequent calls advance toward the policy
        # target at the configured speed.
        now = self._now_timestamp_header()
        if self._last_commanded_joint_positions is None:
            measured, _ = self._read_arm_state()
            self._last_commanded_joint_positions = JointPositions(header=now, positions=measured)
            return measured
        last = self._last_commanded_joint_positions
        dt_s = (now.monotonic_ns - last.header.monotonic_ns) / 1e9
        max_step = self.config.joint_speed_limit_rad_s * dt_s
        delta = target - last.positions
        commanded = last.positions + np.clip(delta, -max_step, max_step)
        self._last_commanded_joint_positions = JointPositions(header=now, positions=commanded)
        return commanded

    def _check_not_halted(self) -> None:
        if self._halted:
            raise RebotB601DmDriverError("write call refused while halted; call resume() first")

    @override
    def write_joint_positions(self, joint_positions: JointPositions) -> None:
        self._check_not_halted()
        self._ensure_arm_mode(Mode.POS_VEL)
        target = joint_positions.positions.astype(np.float64)
        commanded = self._compute_rate_limited_target(target)
        try:
            self._bus.send_arm_pos_vel(commanded)
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc

    @override
    def write_joint_velocities(self, joint_velocities: JointVelocities) -> None:
        self._check_not_halted()
        self._ensure_arm_mode(Mode.VEL)
        try:
            self._bus.send_arm_vel(joint_velocities.velocities.astype(np.float64))
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc

    @override
    def write_ee_positions(self, ee_positions: EEPositions) -> None:
        # The EE vector is the jaw width in metres (see RebotB601DmModel); map to motor radians and send a
        # torque-capped FORCE_POS command. Gripper writes are allowed in any arm mode -- the gripper motor's
        # mode is independent and set once at prime.
        self._check_not_halted()
        if self._current_arm_mode is None:
            raise RebotB601DmDriverError("write call issued before prime(); the arm has no operating mode set")
        motor_rad = gripper_width_to_motor_rad(float(ee_positions.positions[0]))
        try:
            self._bus.send_gripper_force_pos(
                motor_rad=motor_rad,
                torque_ratio=self.config.gripper_torque_ratio,
            )
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc

    @override
    def write_ee_velocities(self, ee_velocities: EEVelocities) -> None:
        # The gripper motor runs FORCE_POS only, so a velocity command maps onto direction semantics:
        # positive width rate drives toward fully open, negative toward fully closed, zero holds the
        # current position. The torque ceiling applies in every case, so a sustained close command grips
        # rather than crushes.
        self._check_not_halted()
        if self._current_arm_mode is None:
            raise RebotB601DmDriverError("write call issued before prime(); the arm has no operating mode set")
        width_rate = float(ee_velocities.velocities[0]) if ee_velocities.velocities.size > 0 else 0.0
        if width_rate > 0.0:
            motor_rad = gripper_width_to_motor_rad(self.model.get_ee_position_limits()[1][0])
        elif width_rate < 0.0:
            motor_rad = gripper_width_to_motor_rad(0.0)
        else:
            current_motor_rad, _ = self._read_gripper_state()
            motor_rad = current_motor_rad
        try:
            self._bus.send_gripper_force_pos(
                motor_rad=motor_rad,
                torque_ratio=self.config.gripper_torque_ratio,
            )
        except MotorBridgeCallError as exc:
            raise RebotB601DmDriverError(str(exc)) from exc
