"""
Standalone hardware experiments against the Ufactory Lite6 over xarm-python-sdk. Each experiment is one
top-level function; the typer CLI at the bottom picks which one to run.

Goal: characterise the SDK surface (return codes, timings, quirks) before wiring it into the aegis hardware
backends. Findings get logged to xarm_api.md at the repo root as we go.

Installed as the lite6_cli console script (see pyproject.toml); run from any shell on a workstation that
can reach the arm:

    lite6_cli stream --ip 192.168.1.178

-h works at every level:

    lite6_cli -h
    lite6_cli stream -h
"""

from __future__ import annotations

import contextlib
import io
import time
from enum import IntEnum
from typing import Annotated, Optional

import numpy as np
import typer

# The xarm SDK prints SDK_VERSION: <ver> to stdout on import with no off switch -- redirect stdout while we
# pull it in so the script's own output isn't preceded by that banner.
with contextlib.redirect_stdout(io.StringIO()):
    from xarm.wrapper import XArmAPI

from manor.manipulators.lite6.joint_configurations import Lite6JointConfiguration

# Default IP that ships from the factory + the deprecated codebase used. Override with --ip per invocation.
DEFAULT_IP = "192.168.1.178"

# Lite6 has 6 actuated arm joints. The xarm SDK pads joint vectors to 7 elements regardless of model; we
# slice down to this many.
LITE6_DOF = 6


class XArmMode(IntEnum):
    """
    xarm SDK mode constants (see xarm/wrapper/xarm_api.py and manor.manipulators.lite6.driver for the
    manor side).
    """

    POSITION = 0  # motion-plan position (set_servo_angle, set_position)
    SERVO_POSITION = 1  # low-latency joint streaming (set_servo_angle_j)
    MANUAL = 2  # joint teaching / manual drag (motors gravity-compensate; user drags by hand)
    VELOCITY = 4  # joint velocity (vc_set_joint_velocity)


class XArmState(IntEnum):
    """
    xarm SDK state constants -- the small subset we actually use from this CLI.
    """

    READY = 0
    STOP = 4


# Default streaming rate for the joint-state dump experiment. Slow enough that the terminal can keep up;
# bump per-invocation if you're scope-watching a fast motion.
DEFAULT_STREAM_HZ = 20.0

# Convergence tolerance for send_joint_positions: max absolute joint-angle error between arm.angles
# (heartbeat-cached measured pose) and the commanded target before we exit the convergence-poll loop.
_SETTLE_TOLERANCE_RAD = 5e-3

# Polling period for the convergence-poll loop in send_joint_positions. arm.angles updates at the
# heartbeat rate (~5 Hz), so anything finer than ~50 ms is wasted work.
_CONVERGE_POLL_PERIOD_S = 0.05


# Hint text appended to the _check error when the SDK returns specific known codes. Keep these short --
# the user's already looking at a traceback; we want the actionable suggestion in the message itself, not
# a wall of explanation.
_CODE_HINTS: dict[int, str] = {
    1: (
        "code=1 ('Not Ready') means the arm is refusing commands. If error_code/warn_code above are "
        "non-zero, a fault latched (force-collision, servo error, etc.) -- check the motion path and try "
        "clean_error + soft motion_enable cycle. If both are zero, the controller itself is wedged: "
        "check e-stop / drag mode, then power-cycle (off, wait 10 s, on)."
    ),
}


def _check(ret_code: int | tuple, op: str, arm: XArmAPI | None = None) -> None:
    """
    Raise a RuntimeError if an xarm SDK call returned a non-zero status code. Some calls return a plain
    int, others return a tuple whose first element is the code; handle both. Known codes get an actionable
    hint appended (see _CODE_HINTS).

    When arm is provided, error_code / warn_code are appended to the message. This is essential for move
    commands: the SDK's _check_code rewrites the firmware return to 1 (Not Ready) any time state_is_ready
    is false at the moment of the call, which can happen because a fault latched between streaming ticks
    (e.g. a force-collision tripping mid-motion). Without error_code / warn_code the real cause is hidden
    behind that opaque code=1.
    """
    code = ret_code[0] if isinstance(ret_code, tuple) else ret_code
    if code == 0:
        return
    msg = f"xarm SDK call {op!r} failed (code={code})"
    if arm is not None:
        msg = f"{msg}, error_code={arm.error_code}, warn_code={arm.warn_code}"
    if hint := _CODE_HINTS.get(code):
        msg = f"{msg}. {hint}"
    raise RuntimeError(msg)


# After motion_enable(True) the brakes release and the servos lock onto current encoder readings; this
# takes ~2 s of micro-motion to settle (observed empirically). Sleep before issuing further state changes
# so set_mode / set_state don't race the bring-up.
_MOTION_ENABLE_SETTLE_S = 2.0

# How long to leave the motors disabled during soft recovery before re-enabling. Long enough that the
# servos fully de-energise so the next motion_enable starts from a known-clean state.
_SOFT_RECOVERY_DISABLE_S = 1.0

# Speed / acceleration for prime + unprime moves via mode 0 set_servo_angle (built-in trajectory
# generation). Deliberately well below firmware ceilings (joint_speed_limit pi rad/s, joint_acc_limit 20
# rad/s^2 from the probe) so the motion is slow enough for the operator to e-stop if anything looks wrong
# while we're still characterising the hardware.
_PRIME_MOVE_SPEED_RAD_S = 1.0
_PRIME_MOVE_ACC_RAD_S2 = 2.0

# Maximum time to wait in _switch_mode for the heartbeat-cached arm.mode to catch up to a recent
# set_mode call. Empirically the report rate is ~5 Hz so this only needs to cover one heartbeat
# interval, but we leave a generous margin since wait_move bailing early on a stale arm.mode silently
# breaks subsequent moves.
_MODE_REPORT_SETTLE_S = 1.0


def prime(arm: XArmAPI, mode: XArmMode = XArmMode.SERVO_POSITION) -> None:
    """
    Bring the arm into a state where reads + writes work, then move it to a known-clear operational pose
    (Lite6JointConfiguration.PRIME).

    Sequence:

    1. clean_warn() + clean_error() to wipe latched faults.
    2. motion_enable(True) turns motors on (audible click).
    3. Sleep _MOTION_ENABLE_SETTLE_S so servos lock onto encoder pose.
    4. set_mode(0) -- always activate in motion-plan position mode so we can use set_servo_angle's
    built-in trajectory generation to move to PRIME.
    5. set_state(0) puts the controller in READY; required before motion calls.
    6. Assert error_code == 0 and warn_code == 0, with _try_soft_recover fallback on failure.
    7. set_servo_angle to Lite6JointConfiguration.PRIME, getting clear of the zero-pose self-collision
    envelope before exposing the arm to operator commands.
    8. Switch to mode if it's not mode 0.

    Step 6 is load-bearing: motion_enable may return success at the controller level while a servo-level
    error is latched on an individual joint (e.g. servo_id=6, code=23 after a previous abrupt unprime).
    Without this check we'd happily proceed and the next motion command would fail with the unhelpful
    code=1 (Not Ready). On failure we attempt a soft motion_enable toggle before raising with a
    power-cycle hint.

    Step 7 deliberately uses mode 0's set_servo_angle (with built-in trajectory generation) rather than
    mode 1's set_servo_angle_j -- mode 1 is the call we're still characterising via the experiment
    commands, so we don't want prime/unprime depending on it. Mode 0 is the canonical, well-understood
    "go to" interface.
    """
    # Always activate in mode 0 (motion-plan position) so step 7 can use set_servo_angle for the move to
    # PRIME. Modes 1 and 4 get switched in at step 8 if that's what the caller asked for.
    _run_prime_sequence(arm, mode=XArmMode.POSITION)
    if arm.error_code != 0 or arm.warn_code != 0:
        typer.echo(
            f"  prime caught error_code={arm.error_code}, warn_code={arm.warn_code}; "
            f"attempting soft recovery (motion_enable off/on cycle)..."
        )
        _try_soft_recover(arm, mode=XArmMode.POSITION)
    if arm.error_code != 0 or arm.warn_code != 0:
        raise RuntimeError(
            f"arm reports error_code={arm.error_code}, warn_code={arm.warn_code} after prime + soft "
            f"recovery. Servo-level errors can survive both clean_error/clean_warn and motion_enable "
            f"cycling -- power-cycle the controller (turn it off, wait a few seconds, turn back on) and "
            f"retry."
        )

    _move_to_configuration(arm, Lite6JointConfiguration.PRIME)

    if mode != XArmMode.POSITION:
        typer.echo(f"  switching from mode {XArmMode.POSITION} to mode {mode}...")
        _switch_mode(arm, mode=mode)


def _run_prime_sequence(arm: XArmAPI, mode: XArmMode) -> None:
    """
    The five-step prime call sequence, factored out so soft recovery can replay it after a motion_enable
    toggle. Each step is checked via _check, so a controller-level failure surfaces immediately;
    servo-level latched errors slip past these returns and only show up via arm.error_code afterward.
    """
    _check(arm.clean_warn(), "clean_warn", arm=arm)
    _check(arm.clean_error(), "clean_error", arm=arm)
    _check(arm.motion_enable(enable=True), "motion_enable", arm=arm)
    time.sleep(_MOTION_ENABLE_SETTLE_S)
    _check(arm.set_mode(mode=mode), f"set_mode({mode})", arm=arm)
    _check(arm.set_state(state=XArmState.READY), "set_state(ready)", arm=arm)


def _try_soft_recover(arm: XArmAPI, mode: XArmMode) -> None:
    """
    Re-cycle motion_enable off-then-on to clear servo-level errors that survive clean_error / clean_warn.
    Best-effort -- we don't _check the intermediate calls because they may legitimately fail mid-recovery;
    the post-recovery arm.error_code read in prime is what decides whether recovery succeeded.

    Not bulletproof: if a servo really won't release its latched error, only physically power-cycling the
    controller will clear it.
    """
    arm.motion_enable(enable=False)
    time.sleep(_SOFT_RECOVERY_DISABLE_S)
    _run_prime_sequence(arm, mode=mode)


def _switch_mode(arm: XArmAPI, mode: XArmMode) -> None:
    """
    Change the active control mode mid-session. The xArm controller requires going through STOP state to
    swap modes; the trio set_state(STOP), set_mode(<new>), set_state(READY) is the canonical sequence.
    Used to flip from the motion-plan position mode that prime always activates into into whatever mode
    the caller actually wants for operation, and back again on unprime.

    After the swap we poll arm.mode until it reflects the new mode. arm.mode is heartbeat-cached and
    lags the set_mode call by ~200ms, and several SDK functions (notably wait_move) check arm.mode
    internally and bail out early if the cached value doesn't match -- e.g. a follow-up
    set_servo_angle(wait=True) would return immediately without waiting if we don't pause for the report
    to catch up, and the next set_state(STOP) would then cancel the un-waited motion.

    Idempotent through redundant STOP/READY cycling -- safe to call when we're already in the target mode
    (the heartbeat poll just returns immediately).
    """
    _check(arm.set_state(state=XArmState.STOP), "set_state(stop)", arm=arm)
    _check(arm.set_mode(mode=mode), f"set_mode({mode})", arm=arm)
    _check(arm.set_state(state=XArmState.READY), "set_state(ready)", arm=arm)
    deadline = time.monotonic() + _MODE_REPORT_SETTLE_S
    while time.monotonic() < deadline and arm.mode != mode:
        time.sleep(0.05)
    if arm.mode != mode:
        typer.echo(
            f"  warning: arm.mode={arm.mode} after set_mode({mode}) within {_MODE_REPORT_SETTLE_S:.1f}s; proceeding"
        )


def _move_to_configuration(arm: XArmAPI, configuration: Lite6JointConfiguration) -> None:
    """
    Move the arm to a named joint configuration via mode 0 set_servo_angle, which has built-in trajectory
    generation and a blocking wait. The arm must already be in mode 0 before calling this; prime activates
    in mode 0 by default, and unprime switches back to mode 0 from whatever the operating command left
    behind.

    Used in preference to mode 1 streaming for prime + unprime because mode 1 (set_servo_angle_j) is the
    call we're still characterising via the experiment commands -- prime/unprime shouldn't depend on it.
    Speed and acceleration are clamped well below firmware ceilings (see _PRIME_MOVE_*).
    """
    angles = configuration.get_joint_positions_vector().tolist()
    typer.echo(f"  moving to {configuration.name} pose...")
    _check(
        arm.set_servo_angle(
            angle=angles,
            speed=_PRIME_MOVE_SPEED_RAD_S,
            mvacc=_PRIME_MOVE_ACC_RAD_S2,
            is_radian=True,
            wait=True,
        ),
        "set_servo_angle",
        arm=arm,
    )


def unprime(arm: XArmAPI) -> None:
    """
    Inverse of prime: move the arm back to Lite6JointConfiguration.ZERO and flip the controller to STOP.
    Does NOT call motion_enable(False) or disconnect -- motors stay energized so the next prime skips the
    brake-release / encoder-relock latency, and the TCP session stays open so we skip the re-handshake.
    Use "lite6_cli disconnect" for full teardown (stop + motor disable + session release) when you're
    done with the session.

    The move-to-ZERO step is wrapped in try/except so a wedged controller (e.g. recovering from a fault
    that the operating command triggered) doesn't block the set_state(STOP) we still want to issue.

    Always switches back to mode 0 first because the move uses set_servo_angle with built-in trajectory
    generation. The switch is harmless when we're already in mode 0 -- safer than checking arm.mode, which
    lags recent set_mode calls via the heartbeat cache.
    """
    try:
        _switch_mode(arm, mode=XArmMode.POSITION)
        _move_to_configuration(arm, Lite6JointConfiguration.ZERO)
    except Exception as exc:
        typer.echo(f"  warning: move-to-{Lite6JointConfiguration.ZERO.name} during unprime failed: {exc}")
    arm.set_state(state=XArmState.STOP)


def read_joint_state(arm: XArmAPI) -> tuple[np.ndarray, np.ndarray]:
    """
    Read positions + velocities in radians / radians-per-second.

    The SDK's get_joint_states returns a 7-element vector for each field regardless of arm DOF (the 7th
    slot is reserved for 7-DOF models); slice to LITE6_DOF here.
    """
    code, raw = arm.get_joint_states(is_radian=True)
    if code != 0:
        raise RuntimeError(f"get_joint_states failed (code={code})")
    positions, velocities, _torques = raw
    return (
        np.asarray(positions, dtype=np.float64)[:LITE6_DOF],
        np.asarray(velocities, dtype=np.float64)[:LITE6_DOF],
    )


# Properties + zero-arg methods to interrogate in probe. Anything that doesn't exist on the connected SDK
# falls back to "<no such attr>" rather than crashing the probe -- the point of this experiment is to
# discover what's exposed without prior knowledge.
_PROBE_PROPERTIES: tuple[str, ...] = (
    # Identity / firmware
    "version",
    "firmware_version",
    "axis",
    "dof",
    "is_simulation_robot",
    # Live state snapshot
    "state",
    "mode",
    "error_code",
    "warn_code",
    "cmdnum",
    # Pose context (handy reference alongside the limits)
    "angles",
    "position",
    # Calibration / offsets
    "tcp_offset",
    "world_offset",
    "tcp_load",
    "gravity_direction",
    # Limits -- the headline of this experiment
    "joint_speed_limit",
    "joint_acc_limit",
    "tcp_speed_limit",
    "tcp_acc_limit",
    # Safety toggles
    "collision_sensitivity",
    "teach_sensitivity",
    "self_collision_detection",
    # Per-joint motor state
    "motor_enable_states",
    "motor_brake_states",
)


def _safe_get(arm: XArmAPI, name: str) -> object:
    """
    Read arm.<name> defensively. Some properties may not exist on the installed SDK version, or may raise
    when the arm is in a particular state -- the probe wants to print something for every entry rather
    than abort.
    """
    try:
        return getattr(arm, name)
    except AttributeError:
        return "<no such attr>"
    except Exception as e:
        return f"<error: {type(e).__name__}: {e}>"


def probe(arm: XArmAPI) -> None:
    """
    Print every property the SDK exposes for the connected arm: identity, live state, pose, calibration,
    limits, and motor state. Pure read -- the caller doesn't prime first so motors stay disabled. Any
    property that returns a sentinel or odd value is itself a useful finding worth logging into
    xarm_api.md.
    """
    for name in _PROBE_PROPERTIES:
        typer.echo(f"  {name:<28} = {_safe_get(arm, name)!r}")


def send_joint_positions(arm: XArmAPI, targets: list[Optional[float]]) -> None:
    """
    Move the arm to targets via mode 1 set_servo_angle_j (the same call Lite6Driver.write_joint_positions
    uses, so this exercises the production code path). Any None entry in targets is replaced with the
    joint's current angle, so -j6 0.5 wiggles joint 6 in isolation.

    Empirically (verified 2026-04-28) a single set_servo_angle_j call is enough -- the firmware servoes
    to the latched target on its own under joint_speed_limit. We just poll arm.angles until it lands
    within _SETTLE_TOLERANCE_RAD of the target. No timeout -- if the arm hasn't converged, Ctrl-C.
    """
    current, _ = read_joint_state(arm)
    resolved = [c if t is None else t for t, c in zip(targets, current, strict=True)]
    target_arr = np.asarray(resolved, dtype=np.float64)
    typer.echo(f"  target: {[f'{v:+0.4f}' for v in resolved]}")
    _check(arm.set_servo_angle_j(angles=resolved, is_radian=True), "set_servo_angle_j", arm=arm)
    while np.max(np.abs(np.asarray(arm.angles, dtype=np.float64)[:LITE6_DOF] - target_arr)) > _SETTLE_TOLERANCE_RAD:
        time.sleep(_CONVERGE_POLL_PERIOD_S)


def send_joint_velocities(arm: XArmAPI, velocities: list[float], duration_s: float) -> None:
    """
    Apply velocities (rad/s, length 6) for duration_s seconds, then zero them. Mode 4 must already be
    active. The zero-out is in a finally so an early Ctrl-C / exception still parks the arm instead of
    leaving the velocity command latched.
    """
    typer.echo(f"  velocities: {[f'{v:+0.4f}' for v in velocities]} for {duration_s:.3f} s")
    try:
        _check(
            arm.vc_set_joint_velocity(speeds=velocities, is_radian=True, duration=0),
            "vc_set_joint_velocity",
            arm=arm,
        )
        time.sleep(duration_s)
    finally:
        _check(
            arm.vc_set_joint_velocity(speeds=[0.0] * LITE6_DOF, is_radian=True, duration=0),
            "vc_set_joint_velocity(zero)",
            arm=arm,
        )


def stream_joint_state(arm: XArmAPI, hz: float, duration_s: Optional[float]) -> None:
    """
    Print joint positions + velocities at hz Hz until either duration_s elapses (if given) or Ctrl-C
    interrupts.
    """
    period = 1.0 / hz
    deadline = None if duration_s is None else time.monotonic() + duration_s
    header_q = "q (rad)"
    header_qdot = "qdot (rad/s)"
    typer.echo(f"streaming at {hz:.1f} Hz (Ctrl-C to stop)...")
    typer.echo(f"  {header_q:<54} {header_qdot}")
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            q, qdot = read_joint_state(arm)
            q_str = " ".join(f"{x:+0.4f}" for x in q)
            qdot_str = " ".join(f"{x:+0.4f}" for x in qdot)
            typer.echo(f"  {q_str:<54} {qdot_str}")
            time.sleep(period)
    except KeyboardInterrupt:
        typer.echo("\ninterrupted.")


# --- CLI --------------------------------------------------------------------

app = typer.Typer(
    add_completion=False,
    help="Standalone hardware experiments against the Lite6 over xarm-python-sdk.",
    context_settings={"help_option_names": ["-h", "--help"]},
)


# Shared option spec so every command spells --ip the same way.
_IP_OPTION = typer.Option("--ip", help="Lite6 robot IP address.")


@app.callback()
def _main() -> None:
    """
    Top-level callback so typer treats this as a multi-command app even when only one @app.command is
    declared. Without it, typer hoists the lone command's args to the top level (e.g. lite6_cli --ip ...
    instead of lite6_cli stream --ip ...), which would break invocations once a second experiment lands.
    """


@app.command("stream")
def cmd_stream(
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
    hz: Annotated[float, typer.Option("--hz", help="Print frequency in Hz.")] = DEFAULT_STREAM_HZ,
    duration: Annotated[
        Optional[float],
        typer.Option(
            "--duration",
            help="How long to stream in seconds. Omit to run until Ctrl-C.",
        ),
    ] = None,
) -> None:
    """
    Connect to the arm, prime it, and continuously print joint positions + velocities. The arm is unprimed
    on exit (Ctrl-C, duration elapsed, or unhandled exception) so motors are released and the TCP session
    is closed.
    """
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    try:
        typer.echo("priming...")
        prime(arm)
        typer.echo("primed.")
        stream_joint_state(arm, hz=hz, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


@app.command("probe")
def cmd_probe(
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Connect to the arm and dump every property the SDK exposes: identity, live state, pose, calibration,
    limits, motor state. Doesn't prime -- pure read; motors stay disabled. Anything printed as
    "<no such attr>" or "<error: ...>" is itself a finding worth recording in xarm_api.md.
    """
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    probe(arm)


@app.command("disconnect")
def cmd_disconnect(
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Full teardown: set_state(STOP), motion_enable(False), disconnect. Other commands deliberately leave
    motors energized + session open after returning so the next prime skips the brake-release /
    encoder-relock latency we see when motion_enable cycles -- run this when you're done with the session
    (or before powering down or handing the arm off). Note: leaving motors energized between commands
    burns a bit of power and warms the servos over time, so end of session = run this.
    """
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    arm.set_state(state=XArmState.STOP)
    arm.motion_enable(enable=False)
    arm.disconnect()
    typer.echo("disconnected (motors disabled, session released).")


@app.command("manual")
def cmd_manual(
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Enter joint teaching (manual drag) mode: motors stay energized but gravity-compensate, so you can
    physically push / pull the arm into any pose by hand. Ctrl-C exits and switches back to mode 0 with
    motors still energized so the arm holds the new pose. Useful for inspecting connections,
    photographing the arm at specific poses, or scouting good operational poses to bake into
    Lite6JointConfiguration.

    Does NOT call prime() (no auto-move to PRIME) -- the whole point is to leave the arm where it
    currently is and let the operator move it. Run lite6_cli disconnect afterward for full teardown if
    you're done with the session.
    """
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    typer.echo("activating motors and entering manual mode...")
    _check(arm.clean_warn(), "clean_warn", arm=arm)
    _check(arm.clean_error(), "clean_error", arm=arm)
    _check(arm.motion_enable(enable=True), "motion_enable", arm=arm)
    time.sleep(_MOTION_ENABLE_SETTLE_S)
    _switch_mode(arm, mode=XArmMode.MANUAL)
    typer.echo("manual mode active. drag the arm freely. press Ctrl-C when done.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        typer.echo("\nexiting manual mode...")
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


@app.command("send_jp")
def cmd_send_jp(
    j1: Annotated[Optional[float], typer.Option("-j1", "--j1", help="Joint 1 target (rad). Default: current.")] = None,
    j2: Annotated[Optional[float], typer.Option("-j2", "--j2", help="Joint 2 target (rad). Default: current.")] = None,
    j3: Annotated[Optional[float], typer.Option("-j3", "--j3", help="Joint 3 target (rad). Default: current.")] = None,
    j4: Annotated[Optional[float], typer.Option("-j4", "--j4", help="Joint 4 target (rad). Default: current.")] = None,
    j5: Annotated[Optional[float], typer.Option("-j5", "--j5", help="Joint 5 target (rad). Default: current.")] = None,
    j6: Annotated[Optional[float], typer.Option("-j6", "--j6", help="Joint 6 target (rad). Default: current.")] = None,
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Move the arm to a joint pose using mode 1 set_servo_angle_j -- the same SDK call
    Lite6Driver.write_joint_positions uses, so this exercises the production code path. Any joint not
    specified on the command line stays at its current angle, so -j6 0.5 wiggles joint 6 in isolation.

    A single set_servo_angle_j call is enough; the firmware servoes to the latched target on its own,
    bounded by joint_speed_limit (pi rad/s, see probe). The CLI just polls arm.angles after the call
    until the pose lands within _SETTLE_TOLERANCE_RAD.
    """
    targets: list[Optional[float]] = [j1, j2, j3, j4, j5, j6]
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    try:
        typer.echo("priming (target mode: servo position)...")
        prime(arm, mode=XArmMode.SERVO_POSITION)
        typer.echo("primed.")
        send_joint_positions(arm, targets=targets)
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


@app.command("send_jv")
def cmd_send_jv(
    duration: Annotated[
        float, typer.Option("-d", "--duration", help="Duration to apply velocity (seconds). Required.")
    ],
    j1: Annotated[float, typer.Option("-j1", "--j1", help="Joint 1 velocity (rad/s).")] = 0.0,
    j2: Annotated[float, typer.Option("-j2", "--j2", help="Joint 2 velocity (rad/s).")] = 0.0,
    j3: Annotated[float, typer.Option("-j3", "--j3", help="Joint 3 velocity (rad/s).")] = 0.0,
    j4: Annotated[float, typer.Option("-j4", "--j4", help="Joint 4 velocity (rad/s).")] = 0.0,
    j5: Annotated[float, typer.Option("-j5", "--j5", help="Joint 5 velocity (rad/s).")] = 0.0,
    j6: Annotated[float, typer.Option("-j6", "--j6", help="Joint 6 velocity (rad/s).")] = 0.0,
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Apply the given joint velocity vector for --duration seconds, then zero the velocity. Uses mode 4
    (joint velocity). Unspecified joints default to zero -- so -j6 0.3 -d 0.5 wiggles joint 6 only, all
    others held still.
    """
    velocities = [j1, j2, j3, j4, j5, j6]
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    try:
        typer.echo("priming (target mode: velocity)...")
        prime(arm, mode=XArmMode.VELOCITY)
        typer.echo("primed.")
        send_joint_velocities(arm, velocities=velocities, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


# typer apps are click apps under the hood; expose the click entry-point as cli so pyproject.toml's
# [project.scripts] can wire lite6_cli to it directly (mirrors aegis_cli's pattern).
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
