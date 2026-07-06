"""
Shared low-level helpers around the motorbridge SDK for the reBot B601 DM.

Both the rebot_b601_dm_cli (the standalone hardware-experiment script) and RebotB601DmDriver (the production
driver consumed by aegis) go through these helpers so the bring-up / tear-down / mode sequences stay in one
place. The control strategy mirrors what the vendor stack (reBotArm_control_py, the ROS2 controller, and the
official LeRobot integration) has validated on this hardware:

* Arm joints run in POS_VEL mode: the DM motor firmware runs a cascaded position/velocity loop with a
per-command velocity limit, using loop gains written into motor registers at bring-up. Streamed position
targets are additionally rate-limited client-side by the driver.
* The gripper ALWAYS runs in FORCE_POS mode: firmware position control with a hard torque ceiling expressed
as a fraction of the motor's maximum torque. This is the critical protection for the 3D printed gripper
linkage -- plain position control (POS_VEL / MIT with stiff gains) applies full motor torque on contact and
has physically broken the printed parts. The vendor's LeRobot integration grips at ratio 0.07.
* Moves to named configurations stream interpolated POS_VEL targets at a bounded joint speed and poll for
convergence, which is how the vendor's safe-home behaves (min-jerk stream to the zero pose at 0.5 rad/s).

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
from manor.manipulators.rebot_b601_dm.model import (
    REBOT_B601_DM_ARM_DOF,
    REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M,
)

LogFn = Callable[[str], None]


def _noop_log(_message: str) -> None:
    pass


# Serial bridge defaults (vendor: Damiao serial bridge enumerating as a CDC ACM device).
REBOT_B601_DM_DEFAULT_CHANNEL = "/dev/ttyACM0"
REBOT_B601_DM_SERIAL_BAUD = 921600

# Gripper motor position at the fully-open jaw state, in radians at the motor. The vendor stack maps motor 0
# rad = fully closed to -5 rad = fully open (rebotarm_hardware.yaml position_limits); the LeRobot integration
# clamps to [-270 deg, 0]. We use the vendor's -5.0 as the full-travel reference for the width mapping and
# treat it as approximate until calibrated against the physical gripper.
REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD = -5.0

# Hard ceiling on the configurable FORCE_POS gripper torque ratio. The vendor's LeRobot integration grips at
# 0.07 (7 percent of max motor torque) and that is plenty to hold objects; anything near 1.0 reproduces the
# full-torque crush that breaks the printed linkage. The config validator refuses ratios above this.
REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX = 0.2

# Default velocity limit (rad/s at the motor) for FORCE_POS gripper commands. Vendor LeRobot default is
# 900 deg/s ~= 15.7 rad/s; we run slower since nothing about our use needs a snappy gripper.
REBOT_B601_DM_GRIPPER_VLIM_RAD_S = 8.0

# Firmware POS_VEL velocity limits per arm joint (rad/s), from the vendor SDK config (rebotarm_dm.yaml):
# 5.0 for the DM-J4340 joints 1-3, 3.0 for the DM-J4310 joints 4-6. These bound how fast the firmware chases
# a streamed position target; the driver additionally rate-limits the target advance client-side.
REBOT_B601_DM_ARM_POS_VEL_VLIM_RAD_S: tuple[float, ...] = (5.0, 5.0, 5.0, 3.0, 3.0, 3.0)

# Velocity limit used for prime / unprime moves to named configurations. Matches the vendor safe-home
# max_vel of 0.5 rad/s -- slow enough for the operator to intervene while we characterise the hardware.
_CONFIGURATION_MOVE_VLIM_RAD_S = 0.5

# Convergence tolerance and polling for move_to_configuration. The vendor safe-home settles at 0.01 rad.
_CONFIGURATION_MOVE_TOLERANCE_RAD = 0.02
_CONFIGURATION_MOVE_POLL_PERIOD_S = 0.05
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
    Map a physical jaw opening width in metres onto the gripper motor position in radians. The mapping is
    linear between fully closed (width 0, motor 0) and fully open (width 0.143, motor -5 rad).
    """
    fraction = width_m / REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M
    return float(np.clip(fraction, 0.0, 1.0)) * REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD


def gripper_motor_rad_to_width(motor_rad: float) -> float:
    """
    Inverse of gripper_width_to_motor_rad; clamps to the physical width range.
    """
    fraction = motor_rad / REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD
    return float(np.clip(fraction, 0.0, 1.0)) * REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M


def gripper_motor_rad_s_to_width_m_s(motor_rad_s: float) -> float:
    """
    Map a gripper motor velocity onto the jaw width rate. Same linear factor as the position mapping; the
    negative motor direction (opening) maps to positive width rate.
    """
    return motor_rad_s * (REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M / REBOT_B601_DM_GRIPPER_MOTOR_OPEN_RAD)


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
        for spec in REBOT_B601_DM_MOTOR_SPECS:
            try:
                self._motor(spec.name).request_feedback()
            except CallError:
                pass
        try:
            self._require_controller().poll_feedback_once()
        except CallError:
            pass

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
        Stream velocity targets to the arm joints. Requires the arm to already be in VEL mode.
        """
        for i, spec in enumerate(self.arm_specs):
            try:
                self._motor(spec.name).send_vel(float(velocities[i]))
            except CallError as exc:
                raise MotorBridgeCallError(f"send_vel failed for motor {spec.name!r}: {exc}") from exc

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

    def set_zero_all(self, log_fn: LogFn = _noop_log) -> None:
        """
        Set the current physical pose as the zero position of every motor. The motors must be disabled
        first (vendor sequence); the operator holds the arm at the vendor home pose (horizontal / sit-down,
        gripper fully closed) while this runs.
        """
        self.disable_all(log_fn=log_fn)
        self._poll_feedback()
        for spec in REBOT_B601_DM_MOTOR_SPECS:
            log_fn(f"zeroing {spec.name}...")
            try:
                self._motor(spec.name).set_zero_position()
            except CallError as exc:
                raise MotorBridgeCallError(f"set_zero_position failed for motor {spec.name!r}: {exc}") from exc
            time.sleep(_PER_MOTOR_SETTLE_S)

    def move_arm_to_configuration(
        self,
        configuration: RebotB601DmJointConfiguration,
        log_fn: LogFn = _noop_log,
    ) -> None:
        """
        Move the arm to a named joint configuration by streaming a POS_VEL target with a slow velocity
        limit and polling for convergence. The firmware's cascade loop does the actual interpolation, so a
        single target with a low vlim yields a smooth bounded-speed move (the vendor safe-home streams a
        min-jerk trajectory at the same effective speed). Requires POS_VEL mode.

        Raises MotorBridgeCallError if the arm has not converged within the timeout -- a wedged or blocked
        move should surface loudly rather than silently proceeding to the next bring-up step.
        """
        target = configuration.get_joint_positions_vector()
        log_fn(f"moving to {configuration.name} pose...")
        vlims = np.full(REBOT_B601_DM_ARM_DOF, _CONFIGURATION_MOVE_VLIM_RAD_S, dtype=np.float64)
        deadline = time.monotonic() + _CONFIGURATION_MOVE_TIMEOUT_S
        self.send_arm_pos_vel(target, vlims=vlims)
        while time.monotonic() < deadline:
            positions, _ = self.read_arm_state()
            if float(np.max(np.abs(positions - target))) < _CONFIGURATION_MOVE_TOLERANCE_RAD:
                return
            # Re-send the target each poll: POS_VEL targets are latched by the firmware, but re-sending
            # costs little and recovers from any dropped frame on the serial bridge.
            self.send_arm_pos_vel(target, vlims=vlims)
            time.sleep(_CONFIGURATION_MOVE_POLL_PERIOD_S)
        positions, _ = self.read_arm_state()
        raise MotorBridgeCallError(
            f"arm did not converge to {configuration.name} within {_CONFIGURATION_MOVE_TIMEOUT_S:.0f}s; "
            f"max error {float(np.max(np.abs(positions - target))):.4f} rad. The arm may be blocked or a "
            f"motor may be faulted -- check the bus and clear errors before retrying."
        )

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
    Full bring-up: open the bus, clear latched errors, enable the motors, configure modes (arm POS_VEL with
    vendor loop gains, gripper FORCE_POS), close the gripper softly, and move the arm to PRIME. Mirrors the
    lite6 prime shape; the gripper close uses the supplied torque ratio so even bring-up can never crush.
    """
    bus.connect(log_fn=log_fn)
    bus.clear_errors(log_fn=log_fn)
    bus.enable_all(log_fn=log_fn)
    bus.set_arm_mode(Mode.POS_VEL, log_fn=log_fn)
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
