"""
Shared low-level helpers around the motorbridge SDK for the reBot B601 DM.

Both the rebot_b601_dm_cli (the standalone hardware-experiment script) and RebotB601DmDriver (the production
driver consumed by aegis) go through these helpers so the bring-up / tear-down / mode sequences stay in one
place. The control strategy mirrors what the vendor stack (reBotArm_control_py, the ROS2 controller, and the
official LeRobot integration) has validated on this hardware:

* Arm joints stream in MIT mode by default (matching the official LeRobot follower's control_mode="mit"):
torque = kp * position_error + kd * velocity_error with NO integrator, so clamping the commanded target
near the measured position hard-bounds the torque and the arm yields on contact. POS_VEL (the DM firmware
cascade with integral terms) is available for collision-checked / planned motion, but its integrators wind
up to full motor torque against an obstacle -- never use it for streamed control near contact.
* The gripper ALWAYS runs in FORCE_POS mode: firmware position control with a hard torque ceiling expressed
as a fraction of the motor's maximum torque. This is the critical protection for the 3D printed gripper
linkage -- plain position control (POS_VEL / MIT with stiff gains) applies full motor torque on contact and
has physically broken the printed parts. The vendor's LeRobot integration grips at ratio 0.07.
* Moves to named configurations run in MIT mode -- the same control law the driver streams in production,
so the cli move and the driver stay on one path (the cli is how that path gets exercised on hardware).
MIT has no native velocity limit (send_mit's velocity arg is a feed-forward setpoint, not a cap), so a
gentle move streams a target that advances at a bounded speed and is clamped within a small band of the
measured position each tick. Torque is kp * clamp with no integrator, so even a blocked move pushes
gently until the timeout aborts it -- it can never wind up the way a POS_VEL move would.

Anything callsite-specific (typer.echo for the cli, ManorLogger for the driver) flows in via an injected
log_fn callable -- the helpers themselves do not import either typer or ManorLogger.

Motor conventions (from the vendor stack): all seven motors sit on one CAN bus behind a Damiao serial bridge
(default /dev/ttyACM0 at 921600 baud). Motor position zero is the vendor home pose (arm horizontal /
sit-down, gripper fully closed); the gripper motor runs 0 (closed) to about -5 rad (fully open).
"""

from __future__ import annotations

import time
from collections.abc import Callable

import attr
import numpy as np
from motorbridge import (
    RID_KI_APR,
    RID_KI_ASR,
    RID_KP_APR,
    RID_KP_ASR,
    CallError,
    Controller,
    Mode,
)

from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.model import REBOT_B601_DM_ARM_DOF

LogFn = Callable[[str], None]


def _noop_log(_message: str) -> None:
    pass


# Serial bridge defaults. The udev rule installed by hardware_system_setup.sh pins the Damiao serial
# bridge (a CDC ACM device) to this stable name regardless of enumeration order; on a host without the
# rule, pass the raw device explicitly (e.g. --channel /dev/ttyACM0 on the CLI).
REBOT_B601_DM_DEFAULT_CHANNEL = "/dev/rebot_b601_dm"
REBOT_B601_DM_SERIAL_BAUD = 921600

# Gripper motor position at the fully-open hard stop, output-side radians, measured on hardware
REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD = -5.9668

# Physical jaw opening at the fully-open stop, measured on hardware (inner parallel faces of the gripper)
REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M = 0.105

# Hard ceiling on the configurable FORCE_POS gripper torque ratio. The vendor's LeRobot integration grips at
# 0.07 (7 percent of max motor torque) and that is plenty to hold objects; anything near 1.0 reproduces the
# full-torque crush that breaks the printed linkage. The config validator refuses ratios above this.
REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX = 0.2

# Hard ceiling on the configurable MIT streaming command-error clamp. The clamp bounds arm torque at
# kp * clamp per joint; 0.3 rad at the streaming kp of 45 is already ~13.5 N*m on joints 1-3 (half the
# DM-J4340's max). The config validator refuses clamps above this.
REBOT_B601_DM_MAX_COMMAND_ERROR_CEILING_RAD = 0.3

# Default velocity limit for FORCE_POS gripper commands, in the rotor-side command frame (~0.8 rad/s at
# the output through the 10:1 gearing, i.e. full stroke in roughly 0.4 s). Hardware-validated 2026-07-06.
REBOT_B601_DM_GRIPPER_VLIM_RAD_S = 8.0

# Firmware POS_VEL velocity limits per arm joint (rad/s), from the vendor SDK config (rebotarm_dm.yaml):
# 5.0 for the DM-J4340 joints 1-3, 3.0 for the DM-J4310 joints 4-6. These bound how fast the firmware chases
# a streamed position target; the driver additionally rate-limits the target advance client-side. Note that
# POS_VEL runs firmware position / velocity loops WITH integral terms, so a blocked joint winds up to full
# motor torque -- POS_VEL is only safe for collision-checked / planned motion, never for streamed control
# near contact. The MIT gains below are the contact-safe alternative.
REBOT_B601_DM_ARM_POS_VEL_VLIM_RAD_S: tuple[float, ...] = (5.0, 5.0, 5.0, 3.0, 3.0, 3.0)

# MIT streaming gains per arm joint for policy / teleop control, from the official LeRobot integration's
# follower defaults (config_rebot_b601_follower.py: control_mode="mit"). Deliberately soft: in MIT mode the
# motor torque is kp * position_error + kd * velocity_error (no integrator), so with the commanded target
# clamped near the measured position the torque is hard-bounded and the arm yields on contact instead of
# breaking its 3D printed structure. This is how the vendor's own teleop drives the arm.
REBOT_B601_DM_ARM_MIT_STREAM_KP: tuple[float, ...] = (45.0, 45.0, 45.0, 8.0, 9.0, 8.0)
REBOT_B601_DM_ARM_MIT_STREAM_KD: tuple[float, ...] = (12.0, 12.0, 12.0, 1.0, 1.0, 1.0)

# MIT gains for bring-up / point-to-point moves (prime / unprime / send_jp), from the vendor ROS2 stack's
# endpos-control MIT gains (rebotarm_hardware.yaml mit_kp / mit_kd). Stiffer than the teleop streaming gains
# above so the arm holds against gravity within the convergence tolerance during a slow move; the
# command-error clamp still bounds torque at about kp * clamp even if the move is blocked.
_MOVE_MIT_KP: tuple[float, ...] = (120.0, 120.0, 120.0, 18.0, 18.0, 18.0)
_MOVE_MIT_KD: tuple[float, ...] = (8.0, 8.0, 8.0, 2.0, 2.0, 2.0)

# MIT configuration-move parameters. MIT has no native velocity limit, so a gentle move streams a target
# that advances toward the goal at the move speed and is additionally clamped within _MOVE_ERROR_CLAMP_RAD
# of the measured position each tick -- the driver's own control law run to completion. A blocked move
# therefore pushes with at most kp * clamp (about 9.6 N*m at joints 1-3, 1.4 N*m at 4-6) until the timeout
# aborts it. REBOT_B601_DM_ARM_CONFIGURATION_MOVE_SPEED_RAD_S is the gentle default advance speed (rad/s);
# send_jp exposes it as --max-speed. The tolerance leaves room for gravity sag under the finite kp.
REBOT_B601_DM_ARM_CONFIGURATION_MOVE_SPEED_RAD_S = 0.5
_CONFIGURATION_MOVE_SEND_RATE_HZ = 50.0
_MOVE_ERROR_CLAMP_RAD = 0.08
_CONFIGURATION_MOVE_TOLERANCE_RAD = 0.05
_CONFIGURATION_MOVE_TIMEOUT_S = 20.0

# Settle pauses copied from the vendor SDK's bring-up sequences: mode transitions and enable/disable are
# fire-and-forget register writes on the bus, and back-to-back writes without a settle have been observed to
# drop (the SDK sleeps 0.05 s per motor after enable / mode calls and 0.2 s after a group mode switch).
_PER_MOTOR_SETTLE_S = 0.05
_GROUP_MODE_SETTLE_S = 0.2
_DISABLE_SETTLE_S = 0.3

# ensure_mode occasionally needs a retry while the motor is still processing the previous register write;
# the vendor LeRobot integration retries up to 9 times with a 0.01 s pause.
_ENSURE_MODE_RETRIES = 9
_ENSURE_MODE_RETRY_PAUSE_S = 0.01
_ENSURE_MODE_TIMEOUT_MS = 1000

# Feedback-poll retry rounds. A single request-all + poll_feedback_once pass only drains the replies that
# have already arrived at the serial bridge -- measured on hardware (2026-07-06), a cold pass catches ~2 of
# the 7 motors and the rest land on the second pass a few milliseconds later. Retry until every motor has
# reported at least once; steady-state reads exit on the first round because motor states stay cached.
_FEEDBACK_RETRY_ROUNDS = 5
_FEEDBACK_RETRY_PAUSE_S = 0.01

# POS_VEL cascade loop gains per motor model, written into the DM motor registers at bring-up. Values come
# from the vendor SDK config (rebotarm_dm.yaml): the register pairs are (velocity kp, velocity ki) on
# RID_KP_ASR / RID_KI_ASR and (position kp, position ki) on RID_KP_APR / RID_KI_APR.
_POS_VEL_GAINS_BY_MODEL: dict[str, tuple[float, float, float, float]] = {
    "4340P": (0.0125, 0.004, 150.0, 0.5),
    "4310": (0.0008, 0.002, 50.0, 1.0),
}


@attr.frozen
class RebotB601DmMotorSpec:
    """
    Identity of one Damiao motor on the reBot B601 DM bus: joint name, CAN send / feedback IDs, and the
    motor model string motorbridge expects. IDs and models come from the vendor SDK config.
    """

    name: str
    send_id: int
    feedback_id: int
    model: str


# Bus layout for the B601 DM: six arm joints then the gripper, ordered to match the URDF joint order.
REBOT_B601_DM_MOTOR_SPECS: tuple[RebotB601DmMotorSpec, ...] = (
    RebotB601DmMotorSpec(name="joint1", send_id=0x01, feedback_id=0x11, model="4340P"),
    RebotB601DmMotorSpec(name="joint2", send_id=0x02, feedback_id=0x12, model="4340P"),
    RebotB601DmMotorSpec(name="joint3", send_id=0x03, feedback_id=0x13, model="4340P"),
    RebotB601DmMotorSpec(name="joint4", send_id=0x04, feedback_id=0x14, model="4310"),
    RebotB601DmMotorSpec(name="joint5", send_id=0x05, feedback_id=0x15, model="4310"),
    RebotB601DmMotorSpec(name="joint6", send_id=0x06, feedback_id=0x16, model="4310"),
    RebotB601DmMotorSpec(name="gripper", send_id=0x07, feedback_id=0x17, model="4310"),
)


class MotorBridgeCallError(RuntimeError):
    """
    Raised when a motorbridge call fails. Wraps the SDK's CallError with the operation name so callers can
    tell which step of a sequence died.
    """


def gripper_width_to_motor_rad(width_m: float) -> float:
    """
    Map a physical jaw opening width in metres onto a gripper FORCE_POS command position: linear between
    fully closed (width 0, command 0) and the measured fully-open stop (width
    REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M, command REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD).
    """
    fraction = width_m / REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M
    return float(np.clip(fraction, 0.0, 1.0)) * REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD


def gripper_motor_rad_to_width(motor_rad: float) -> float:
    """
    Inverse of gripper_width_to_motor_rad (commands and feedback share one frame); clamps to the
    physical width range.
    """
    fraction = motor_rad / REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD
    return float(np.clip(fraction, 0.0, 1.0)) * REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M


def gripper_motor_rad_s_to_width_m_s(motor_rad_s: float) -> float:
    """
    Map a gripper motor velocity onto the jaw width rate. Same linear factor as the position mapping;
    negative motor velocity (opening) maps to positive width rate.
    """
    return motor_rad_s * (REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M / REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD)


@attr.define
class RebotB601DmBus:
    """
    Thin ownership wrapper for the B601 DM motor bus: the motorbridge Controller plus the per-joint Motor
    handles, with the bring-up / tear-down / mode sequences validated against the vendor stack.

    Construction is cheap and side-effect free (no serial port is opened); connect() builds the Controller
    and registers the motors. This mirrors the lite6 driver's do_not_open pattern so unit tests and dry
    imports never touch hardware.
    """

    channel: str = REBOT_B601_DM_DEFAULT_CHANNEL
    _controller: Controller | None = attr.field(init=False, default=None)
    _motors: dict[str, object] = attr.field(init=False, factory=dict)

    @property
    def connected(self) -> bool:
        return self._controller is not None

    @property
    def arm_specs(self) -> tuple[RebotB601DmMotorSpec, ...]:
        return REBOT_B601_DM_MOTOR_SPECS[:REBOT_B601_DM_ARM_DOF]

    @property
    def gripper_spec(self) -> RebotB601DmMotorSpec:
        return REBOT_B601_DM_MOTOR_SPECS[REBOT_B601_DM_ARM_DOF]

    def _motor(self, name: str):
        if not self.connected:
            raise MotorBridgeCallError(f"motor {name!r} accessed before connect()")
        return self._motors[name]

    def connect(self, log_fn: LogFn = _noop_log) -> None:
        """
        Open the serial bridge and register every motor. Idempotent -- a second call is a no-op so prime()
        can run repeatedly on the same bus instance.
        """
        if self.connected:
            return
        log_fn(f"opening serial bridge on {self.channel}...")
        try:
            controller = Controller.from_dm_serial(self.channel, REBOT_B601_DM_SERIAL_BAUD)
            motors: dict[str, object] = {}
            for spec in REBOT_B601_DM_MOTOR_SPECS:
                motors[spec.name] = controller.add_damiao_motor(spec.send_id, spec.feedback_id, spec.model)
        except CallError as exc:
            raise MotorBridgeCallError(f"failed to open bus / register motors on {self.channel}: {exc}") from exc
        self._controller = controller
        self._motors = motors

    def disconnect(self) -> None:
        """
        Disable the motors and release the serial bridge. Safe to call when never connected.
        """
        if not self.connected:
            return
        try:
            self._controller.disable_all()
            time.sleep(_DISABLE_SETTLE_S)
            self._controller.shutdown()
            self._controller.close()
        except CallError:
            pass
        self._controller = None
        self._motors = {}

    def enable_all(self, log_fn: LogFn = _noop_log) -> None:
        log_fn("enabling motors...")
        try:
            self._require_controller().enable_all()
        except CallError as exc:
            raise MotorBridgeCallError(f"enable_all failed: {exc}") from exc
        time.sleep(_PER_MOTOR_SETTLE_S)

    def disable_all(self, log_fn: LogFn = _noop_log) -> None:
        log_fn("disabling motors...")
        try:
            self._require_controller().disable_all()
        except CallError as exc:
            raise MotorBridgeCallError(f"disable_all failed: {exc}") from exc
        time.sleep(_DISABLE_SETTLE_S)

    def clear_errors(self, log_fn: LogFn = _noop_log) -> None:
        """
        Clear latched per-motor error states (over-temperature, over-current, communication loss). Motors
        refuse enable while an error is latched, so bring-up runs this first.
        """
        log_fn("clearing latched motor errors...")
        for spec in REBOT_B601_DM_MOTOR_SPECS:
            try:
                self._motor(spec.name).clear_error()
            except CallError:
                # Best effort: a motor that never faulted may nack the clear; the post-bring-up status
                # check is what decides whether the bus is healthy.
                pass
            time.sleep(_ENSURE_MODE_RETRY_PAUSE_S)

    def _ensure_motor_mode(self, name: str, mode: Mode) -> None:
        motor = self._motor(name)
        for attempt in range(_ENSURE_MODE_RETRIES + 1):
            try:
                motor.ensure_mode(mode, _ENSURE_MODE_TIMEOUT_MS)
                return
            except CallError as exc:
                if attempt == _ENSURE_MODE_RETRIES:
                    raise MotorBridgeCallError(f"ensure_mode({mode!r}) failed for motor {name!r}: {exc}") from exc
                time.sleep(_ENSURE_MODE_RETRY_PAUSE_S)

    def set_arm_mode(self, mode: Mode, log_fn: LogFn = _noop_log) -> None:
        """
        Switch every arm joint to the given control mode. POS_VEL transitions also (re)write the cascade
        loop gain registers first, following the vendor SDK's mode_pos_vel sequence.
        """
        log_fn(f"switching arm to mode {mode!r}...")
        for spec in self.arm_specs:
            if mode == Mode.POS_VEL:
                self._write_pos_vel_gains(spec)
            self._ensure_motor_mode(spec.name, mode)
            time.sleep(_PER_MOTOR_SETTLE_S)
        time.sleep(_GROUP_MODE_SETTLE_S)

    def set_gripper_mode_force_pos(self, log_fn: LogFn = _noop_log) -> None:
        """
        Put the gripper motor into FORCE_POS mode: firmware position control with a per-command torque
        ceiling. This is the only mode the production stack drives the gripper in.
        """
        log_fn("switching gripper to FORCE_POS mode...")
        self._ensure_motor_mode(self.gripper_spec.name, Mode.FORCE_POS)
        time.sleep(_GROUP_MODE_SETTLE_S)

    def _write_pos_vel_gains(self, spec: RebotB601DmMotorSpec) -> None:
        vel_kp, vel_ki, pos_kp, pos_ki = _POS_VEL_GAINS_BY_MODEL[spec.model]
        motor = self._motor(spec.name)
        try:
            motor.write_register_f32(RID_KP_ASR, vel_kp)
            motor.write_register_f32(RID_KI_ASR, vel_ki)
            motor.write_register_f32(RID_KP_APR, pos_kp)
            motor.write_register_f32(RID_KI_APR, pos_ki)
        except CallError as exc:
            raise MotorBridgeCallError(f"writing POS_VEL gains failed for motor {spec.name!r}: {exc}") from exc
        time.sleep(_ENSURE_MODE_RETRY_PAUSE_S)

    def _poll_feedback(self) -> None:
        # Request-and-poll until every motor has reported at least once (bounded retries): the serial
        # bridge delivers replies over a few milliseconds, so a single poll pass right after the requests
        # misses whichever motors have not answered yet. Once a motor has reported, its state stays cached
        # and later rounds are unnecessary, so warm reads exit after the first pass.
        for round_index in range(_FEEDBACK_RETRY_ROUNDS):
            for spec in REBOT_B601_DM_MOTOR_SPECS:
                try:
                    self._motor(spec.name).request_feedback()
                except CallError:
                    pass
            try:
                self._require_controller().poll_feedback_once()
            except CallError:
                pass
            if all(self._motor(spec.name).get_state() is not None for spec in REBOT_B601_DM_MOTOR_SPECS):
                return
            if round_index < _FEEDBACK_RETRY_ROUNDS - 1:
                time.sleep(_FEEDBACK_RETRY_PAUSE_S)

    def read_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Read (positions, velocities, torques) for all seven motors (arm joints then gripper), each as a
        float64 array in motor radians / rad/s / N*m. Motors that have not reported yet read as zero.
        """
        self._poll_feedback()
        positions = np.zeros(len(REBOT_B601_DM_MOTOR_SPECS), dtype=np.float64)
        velocities = np.zeros(len(REBOT_B601_DM_MOTOR_SPECS), dtype=np.float64)
        torques = np.zeros(len(REBOT_B601_DM_MOTOR_SPECS), dtype=np.float64)
        for i, spec in enumerate(REBOT_B601_DM_MOTOR_SPECS):
            state = self._motor(spec.name).get_state()
            if state is not None:
                positions[i] = state.pos
                velocities[i] = state.vel
                torques[i] = state.torq
        return positions, velocities, torques

    def read_arm_state(self) -> tuple[np.ndarray, np.ndarray]:
        positions, velocities, _ = self.read_state()
        return positions[:REBOT_B601_DM_ARM_DOF], velocities[:REBOT_B601_DM_ARM_DOF]

    def read_gripper_state(self) -> tuple[float, float]:
        positions, velocities, _ = self.read_state()
        return float(positions[REBOT_B601_DM_ARM_DOF]), float(velocities[REBOT_B601_DM_ARM_DOF])

    def send_arm_pos_vel(self, positions: np.ndarray, vlims: np.ndarray | None = None) -> None:
        """
        Stream POS_VEL position targets to the arm joints. vlims defaults to the per-joint firmware limits
        from the vendor config. Requires the arm to already be in POS_VEL mode.
        """
        if vlims is None:
            vlims = np.asarray(REBOT_B601_DM_ARM_POS_VEL_VLIM_RAD_S, dtype=np.float64)
        for i, spec in enumerate(self.arm_specs):
            try:
                self._motor(spec.name).send_pos_vel(float(positions[i]), float(vlims[i]))
            except CallError as exc:
                raise MotorBridgeCallError(f"send_pos_vel failed for motor {spec.name!r}: {exc}") from exc

    def send_arm_vel(self, velocities: np.ndarray) -> None:
        """
        Stream velocity targets to the arm joints. Requires the arm to already be in VEL mode. Note the
        firmware velocity loop has an integral term, so a blocked joint winds up to full torque -- prefer
        send_arm_mit with kp = 0 for contact-safe velocity control.
        """
        for i, spec in enumerate(self.arm_specs):
            try:
                self._motor(spec.name).send_vel(float(velocities[i]))
            except CallError as exc:
                raise MotorBridgeCallError(f"send_vel failed for motor {spec.name!r}: {exc}") from exc

    def send_arm_mit(
        self,
        positions: np.ndarray,
        velocities: np.ndarray | None = None,
        kp: np.ndarray | None = None,
        kd: np.ndarray | None = None,
    ) -> None:
        """
        Stream MIT impedance commands to the arm joints: per-joint torque = kp * (p_des - p) +
        kd * (v_des - v), saturated at the motor's max torque. No integrator, so bounding the commanded
        position error bounds the torque. Requires the arm to already be in MIT mode. kp / kd default to
        the LeRobot streaming gains; velocities default to zero.
        """
        if velocities is None:
            velocities = np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64)
        if kp is None:
            kp = np.asarray(REBOT_B601_DM_ARM_MIT_STREAM_KP, dtype=np.float64)
        if kd is None:
            kd = np.asarray(REBOT_B601_DM_ARM_MIT_STREAM_KD, dtype=np.float64)
        for i, spec in enumerate(self.arm_specs):
            try:
                self._motor(spec.name).send_mit(
                    float(positions[i]),
                    float(velocities[i]),
                    float(kp[i]),
                    float(kd[i]),
                    0.0,
                )
            except CallError as exc:
                raise MotorBridgeCallError(f"send_mit failed for motor {spec.name!r}: {exc}") from exc

    def send_gripper_force_pos(
        self,
        motor_rad: float,
        torque_ratio: float,
        vlim_rad_s: float = REBOT_B601_DM_GRIPPER_VLIM_RAD_S,
    ) -> None:
        """
        Command the gripper in FORCE_POS mode: chase motor_rad at up to vlim_rad_s, never exceeding
        torque_ratio of the motor's maximum torque. torque_ratio is validated at the config layer; this
        helper re-clamps defensively so no code path can command a crush.
        """
        ratio = float(np.clip(torque_ratio, 0.0, REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX))
        try:
            self._motor(self.gripper_spec.name).send_force_pos(float(motor_rad), float(vlim_rad_s), ratio)
        except CallError as exc:
            raise MotorBridgeCallError(f"send_force_pos failed for gripper: {exc}") from exc

    def set_zero(self, joint_names: list[str] | None = None, log_fn: LogFn = _noop_log) -> None:
        """
        Set the current physical pose as the zero position of the named motors (all seven when
        joint_names is None). The motors must be disabled first (vendor sequence) -- note this disables
        the WHOLE bus, so the backdrivable arm must be resting (or supported) even for a single-joint
        zero. The operator holds the target joints at their zero pose while this runs: the vendor home
        pose for the arm, fully closed for the gripper.
        """
        known = {spec.name for spec in REBOT_B601_DM_MOTOR_SPECS}
        if joint_names is None:
            specs = list(REBOT_B601_DM_MOTOR_SPECS)
        else:
            unknown = sorted(set(joint_names) - known)
            if unknown:
                raise MotorBridgeCallError(f"unknown joint names {unknown}; expected a subset of {sorted(known)}")
            specs = [spec for spec in REBOT_B601_DM_MOTOR_SPECS if spec.name in set(joint_names)]
        self.disable_all(log_fn=log_fn)
        self._poll_feedback()
        for spec in specs:
            log_fn(f"zeroing {spec.name}...")
            try:
                self._motor(spec.name).set_zero_position()
            except CallError as exc:
                raise MotorBridgeCallError(f"set_zero_position failed for motor {spec.name!r}: {exc}") from exc
            time.sleep(_PER_MOTOR_SETTLE_S)

    def move_arm_to(
        self,
        target: np.ndarray,
        label: str,
        speed_rad_s: float = REBOT_B601_DM_ARM_CONFIGURATION_MOVE_SPEED_RAD_S,
        log_fn: LogFn = _noop_log,
    ) -> None:
        """
        Move the arm to a joint-position target in MIT mode -- the same control law the driver streams in
        production. MIT has no native velocity limit, so a gentle move streams a target that advances toward
        the goal at speed_rad_s and is clamped within _MOVE_ERROR_CLAMP_RAD of the measured position each
        tick. This loop is a trajectory generator: it turns a far target into a run of near targets, which
        is exactly what a human hand supplies for free during teleop, so the per-tick error (and hence the
        torque) stays small. Because MIT torque is kp * error with no integrator, a blocked move pushes with
        at most kp * clamp until the timeout aborts it. Switches the arm to MIT mode itself and leaves it
        holding the target; the caller restores its preferred operating mode afterwards if different.

        Raises MotorBridgeCallError if the arm has not converged within the timeout -- a wedged or blocked
        move should surface loudly rather than silently proceeding to the next bring-up step.
        """
        target = np.asarray(target, dtype=np.float64)
        log_fn(f"moving to {label} pose (MIT, <= {speed_rad_s:.2f} rad/s)...")
        self.set_arm_mode(Mode.MIT, log_fn=log_fn)
        kp = np.asarray(_MOVE_MIT_KP, dtype=np.float64)
        kd = np.asarray(_MOVE_MIT_KD, dtype=np.float64)
        dt = 1.0 / _CONFIGURATION_MOVE_SEND_RATE_HZ
        step = speed_rad_s * dt
        positions, _ = self.read_arm_state()
        interpolant = positions.copy()
        deadline = time.monotonic() + _CONFIGURATION_MOVE_TIMEOUT_S
        while time.monotonic() < deadline:
            positions, _ = self.read_arm_state()
            if float(np.max(np.abs(positions - target))) < _CONFIGURATION_MOVE_TOLERANCE_RAD:
                # Latch the exact target as the hold command before returning.
                self.send_arm_mit(target, kp=kp, kd=kd)
                return
            interpolant += np.clip(target - interpolant, -step, step)
            # Torque bound: never let the commanded position lead the measured one by more than the clamp,
            # no matter how far the interpolant has advanced.
            commanded = positions + np.clip(interpolant - positions, -_MOVE_ERROR_CLAMP_RAD, _MOVE_ERROR_CLAMP_RAD)
            self.send_arm_mit(commanded, kp=kp, kd=kd)
            time.sleep(dt)
        positions, _ = self.read_arm_state()
        # Hold wherever the arm actually is rather than keep pushing toward the unreachable target.
        self.send_arm_mit(positions, kp=kp, kd=kd)
        raise MotorBridgeCallError(
            f"arm did not converge to {label} within {_CONFIGURATION_MOVE_TIMEOUT_S:.0f}s; max error "
            f"{float(np.max(np.abs(positions - target))):.4f} rad. The arm may be blocked or a motor may be "
            f"faulted -- now holding the current pose; check the bus and clear errors before retrying."
        )

    def move_arm_to_configuration(
        self,
        configuration: RebotB601DmJointConfiguration,
        log_fn: LogFn = _noop_log,
    ) -> None:
        """
        move_arm_to for a named joint configuration.
        """
        self.move_arm_to(configuration.get_joint_positions_vector(), configuration.name, log_fn=log_fn)

    def _require_controller(self) -> Controller:
        if self._controller is None:
            raise MotorBridgeCallError("bus used before connect()")
        return self._controller


def prime(
    bus: RebotB601DmBus,
    gripper_torque_ratio: float,
    log_fn: LogFn = _noop_log,
) -> None:
    """
    Full bring-up: open the bus, clear latched errors, enable the motors, put the gripper in FORCE_POS and
    close it softly, then move the arm to PRIME via the torque-bounded MIT move. Mirrors the lite6 prime
    shape; the gripper is torque-capped and the arm move runs at the gentle bring-up speed with a bounded
    command error. The arm is left in MIT mode holding PRIME -- the caller switches its mode if different.
    """
    bus.connect(log_fn=log_fn)
    bus.clear_errors(log_fn=log_fn)
    bus.enable_all(log_fn=log_fn)
    bus.set_gripper_mode_force_pos(log_fn=log_fn)
    bus.send_gripper_force_pos(motor_rad=0.0, torque_ratio=gripper_torque_ratio)
    bus.move_arm_to_configuration(RebotB601DmJointConfiguration.PRIME, log_fn=log_fn)


def unprime(bus: RebotB601DmBus, gripper_torque_ratio: float, log_fn: LogFn = _noop_log) -> None:
    """
    Inverse of prime: close the gripper, move the arm back to REST (the vendor home pose, where the folded
    arm is safe to de-energize), then disable the motors. The serial bridge stays open so a subsequent
    prime on the same bus instance skips the reconnect.

    The move-to-REST step is wrapped in try/except so a wedged bus still gets the motor disable -- leaving
    the motors energized and unattended is worse than skipping the park move.
    """
    try:
        bus.send_gripper_force_pos(motor_rad=0.0, torque_ratio=gripper_torque_ratio)
        bus.move_arm_to_configuration(RebotB601DmJointConfiguration.REST, log_fn=log_fn)
    except MotorBridgeCallError as exc:
        log_fn(f"warning: move-to-{RebotB601DmJointConfiguration.REST.name} during unprime failed: {exc}")
    bus.disable_all(log_fn=log_fn)
