"""
Shared low-level helpers around the xarm-python-sdk for the Lite6.

Both the lite6_cli (the standalone hardware-experiment script) and Lite6Driver (the production driver
consumed by aegis) go through these helpers so the prime / unprime / mode-switch sequences stay in one place.
The cli previously owned this logic exclusively while we characterised the SDK; the driver now imports the
same primitives so hardware behaviour matches what the cli has been validated against.

Anything callsite-specific (typer.echo for the cli, ManorLogger for the driver) flows in via an injected
log_fn callable -- the helpers themselves do not import either typer or ManorLogger.
"""

from __future__ import annotations

import contextlib
import io
import time
from collections.abc import Callable
from enum import IntEnum

# The xarm SDK prints SDK_VERSION: <ver> to stdout on import with no off switch -- swallow it during the
# import so importing this module (transitively dragged in by Lite6Driver, lite6_cli, and any unit test
# that touches them) doesn't leak a banner into every CLI's output.
with contextlib.redirect_stdout(io.StringIO()):
    from xarm.wrapper import XArmAPI

from manor.manipulators.lite6.joint_configurations import Lite6JointConfiguration

# Default log sink: ignore every call. Callers that want to surface progress messages pass their own log_fn
# (typer.echo, ManorLogger.info, print, etc.). Keeping the default a no-op means the helpers are usable from
# contexts where a logger isn't wired up yet.
LogFn = Callable[[str], None]


def _noop_log(_message: str) -> None:
    pass


class XArmMode(IntEnum):
    """
    xarm SDK mode constants (subset we use).
    """

    POSITION = 0  # motion-plan position (set_servo_angle, set_position)
    SERVO_POSITION = 1  # low-latency joint streaming (set_servo_angle_j)
    MANUAL = 2  # joint teaching / manual drag (motors gravity-compensate)
    VELOCITY = 4  # joint velocity (vc_set_joint_velocity)


class XArmState(IntEnum):
    """
    xarm SDK state constants (subset we use).
    """

    READY = 0
    STOP = 4


# After motion_enable(True) the brakes release and the servos lock onto current encoder readings; this takes
# ~2 s of micro-motion to settle (observed empirically). Sleep before issuing further state changes so
# set_mode / set_state don't race the bring-up.
_MOTION_ENABLE_SETTLE_S = 2.0

# How long to leave the motors disabled during soft recovery before re-enabling. Long enough that the servos
# fully de-energise so the next motion_enable starts from a known-clean state.
_SOFT_RECOVERY_DISABLE_S = 1.0

# Speed / acceleration for prime + unprime moves via mode 0 set_servo_angle (built-in trajectory generation).
# Deliberately well below firmware ceilings (joint_speed_limit pi rad/s, joint_acc_limit 20 rad/s^2 from the
# probe) so the motion is slow enough for the operator to e-stop if anything looks wrong while we're still
# characterising the hardware.
_PRIME_MOVE_SPEED_RAD_S = 1.0
_PRIME_MOVE_ACC_RAD_S2 = 2.0

# Maximum time to wait in switch_mode for the heartbeat-cached arm.mode to catch up to a recent set_mode
# call. Empirically the report rate is ~5 Hz so this only needs to cover one heartbeat interval, but we leave
# a generous margin since wait_move bailing early on a stale arm.mode silently breaks subsequent moves.
_MODE_REPORT_SETTLE_S = 1.0

# Hint text appended to the check_xarm_call error when the SDK returns specific known codes. Keep these short
# -- the user's already looking at a traceback; we want the actionable suggestion in the message itself, not
# a wall of explanation.
_CODE_HINTS: dict[int, str] = {
    1: (
        "code=1 ('Not Ready') means the arm is refusing commands. If error_code/warn_code above are "
        "non-zero, a fault latched (force-collision, servo error, etc.) -- check the motion path and try "
        "clean_error + soft motion_enable cycle. If both are zero, the controller itself is wedged: "
        "check e-stop / drag mode, then power-cycle (off, wait 10 s, on)."
    ),
}


class XArmCallError(RuntimeError):
    """
    Raised when an xarm SDK call returns a non-zero status code.
    """


def check_xarm_call(ret_code: int | tuple, op: str, arm: XArmAPI | None = None) -> None:
    """
    Raise XArmCallError if an xarm SDK call returned a non-zero status code. Some calls return a plain
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
    raise XArmCallError(msg)


def run_prime_sequence(arm: XArmAPI, mode: XArmMode) -> None:
    """
    The five-step prime call sequence, factored out so soft recovery can replay it after a motion_enable
    toggle. Each step is checked via check_xarm_call, so a controller-level failure surfaces immediately;
    servo-level latched errors slip past these returns and only show up via arm.error_code afterward.
    """
    check_xarm_call(arm.clean_warn(), "clean_warn", arm=arm)
    check_xarm_call(arm.clean_error(), "clean_error", arm=arm)
    check_xarm_call(arm.motion_enable(enable=True), "motion_enable", arm=arm)
    time.sleep(_MOTION_ENABLE_SETTLE_S)
    check_xarm_call(arm.set_mode(mode=mode), f"set_mode({mode})", arm=arm)
    check_xarm_call(arm.set_state(state=XArmState.READY), "set_state(ready)", arm=arm)


def try_soft_recover(arm: XArmAPI, mode: XArmMode) -> None:
    """
    Re-cycle motion_enable off-then-on to clear servo-level errors that survive clean_error / clean_warn.
    Best-effort -- we don't check the intermediate calls because they may legitimately fail mid-recovery;
    the post-recovery arm.error_code read in connect is what decides whether recovery succeeded.

    Not bulletproof: if a servo really won't release its latched error, only physically power-cycling the
    controller will clear it.
    """
    arm.motion_enable(enable=False)
    time.sleep(_SOFT_RECOVERY_DISABLE_S)
    run_prime_sequence(arm, mode=mode)


def connect(arm: XArmAPI, log_fn: LogFn = _noop_log) -> None:
    """
    Bring the controller to a state where reads + writes work: clear latched faults, energize the motors,
    activate mode 0 (motion-plan position) + state READY, and verify a clean error/warn code with a soft
    recovery fallback. No motion is commanded -- the arm stays where it currently is.

    Step ordering matches the cli's connect command (which has been validated on real hardware).

    Two recovery branches handle the two ways latched faults manifest:

    1. The first call in run_prime_sequence (clean_warn) returns code=1 (Not Ready) because error_code is
    non-zero -- the arm is refusing commands outright. The bring-up sequence raises partway through, so we
    never reach a successful end state to inspect. Catch the XArmCallError, attempt try_soft_recover (which
    cycles motion_enable off/on and replays the sequence), and continue.
    2. The sequence completes but arm.error_code / warn_code is still non-zero -- a servo-level error latched
    silently while the controller-level calls returned success (e.g. servo_id=6, code=23 after a previous
    abrupt unprime). The post-bring-up check catches this and runs the same try_soft_recover path.

    Both paths re-check error_code / warn_code afterwards; if either is still non-zero we raise with a
    power-cycle hint since neither soft path can clear servo errors that need a physical reset.
    """
    try:
        run_prime_sequence(arm, mode=XArmMode.POSITION)
    except XArmCallError as exc:
        log_fn(
            f"connect's bring-up sequence failed mid-call ({exc}); the arm is likely refusing "
            f"commands due to a latched fault. Attempting soft recovery (motion_enable off/on cycle)..."
        )
        try_soft_recover(arm, mode=XArmMode.POSITION)
    if arm.error_code != 0 or arm.warn_code != 0:
        log_fn(
            f"connect caught error_code={arm.error_code}, warn_code={arm.warn_code} after bring-up; "
            f"attempting soft recovery (motion_enable off/on cycle)..."
        )
        try_soft_recover(arm, mode=XArmMode.POSITION)
    if arm.error_code != 0 or arm.warn_code != 0:
        raise XArmCallError(
            f"arm reports error_code={arm.error_code}, warn_code={arm.warn_code} after connect + soft "
            f"recovery. Servo-level errors can survive both clean_error/clean_warn and motion_enable "
            f"cycling -- power-cycle the controller (turn it off, wait a few seconds, turn back on) and "
            f"retry."
        )


def switch_mode(arm: XArmAPI, mode: XArmMode, log_fn: LogFn = _noop_log) -> None:
    """
    Change the active control mode mid-session. The xArm controller requires going through STOP state to
    swap modes; the trio set_state(STOP), set_mode(<new>), set_state(READY) is the canonical sequence.
    Used to flip from the motion-plan position mode that connect always activates into into whatever mode
    the caller actually wants for operation, and back again on unprime.

    After the swap we poll arm.mode until it reflects the new mode. arm.mode is heartbeat-cached and
    lags the set_mode call by ~200ms, and several SDK functions (notably wait_move) check arm.mode
    internally and bail out early if the cached value doesn't match -- e.g. a follow-up
    set_servo_angle(wait=True) would return immediately without waiting if we don't pause for the report
    to catch up, and the next set_state(STOP) would then cancel the un-waited motion.

    Idempotent through redundant STOP/READY cycling -- safe to call when we're already in the target mode
    (the heartbeat poll just returns immediately).
    """
    check_xarm_call(arm.set_state(state=XArmState.STOP), "set_state(stop)", arm=arm)
    check_xarm_call(arm.set_mode(mode=mode), f"set_mode({mode})", arm=arm)
    check_xarm_call(arm.set_state(state=XArmState.READY), "set_state(ready)", arm=arm)
    deadline = time.monotonic() + _MODE_REPORT_SETTLE_S
    while time.monotonic() < deadline and arm.mode != mode:
        time.sleep(0.05)
    if arm.mode != mode:
        log_fn(f"warning: arm.mode={arm.mode} after set_mode({mode}) within {_MODE_REPORT_SETTLE_S:.1f}s; proceeding")


def move_to_configuration(arm: XArmAPI, configuration: Lite6JointConfiguration, log_fn: LogFn = _noop_log) -> None:
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
    log_fn(f"moving to {configuration.name} pose...")
    check_xarm_call(
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


def prime(arm: XArmAPI, log_fn: LogFn = _noop_log) -> None:
    """
    Connect the arm (motors energized, mode 0 / READY, errors clean -- see connect) and move it to a
    known-clear operational pose (Lite6JointConfiguration.PRIME). Always leaves the arm in mode 0
    (POSITION); the operating mode for streaming commands is the caller's responsibility -- typically
    Lite6Driver flips into SERVO_POSITION / VELOCITY lazily on the first write.

    The move-to-PRIME step uses mode 0's set_servo_angle (with built-in trajectory generation) rather
    than mode 1's set_servo_angle_j: mode 0 is the canonical "go to" interface and the move-to-PRIME
    helper requires that mode.
    """
    connect(arm, log_fn=log_fn)
    move_to_configuration(arm, Lite6JointConfiguration.PRIME, log_fn=log_fn)


def unprime(arm: XArmAPI, log_fn: LogFn = _noop_log) -> None:
    """
    Inverse of prime: move the arm back to Lite6JointConfiguration.REST and flip the controller to STOP.
    Does NOT call motion_enable(False) or disconnect -- motors stay energized so the next prime skips the
    brake-release / encoder-relock latency, and the TCP session stays open so we skip the re-handshake.
    Use the lite6_cli disconnect command for full teardown (stop + motor disable + session release) when
    you're done with the session.

    The move-to-REST step is wrapped in try/except so a wedged controller (e.g. recovering from a fault
    that the operating command triggered) doesn't block the set_state(STOP) we still want to issue.

    Always switches back to mode 0 first because the move uses set_servo_angle with built-in trajectory
    generation. The switch is harmless when we're already in mode 0 -- safer than checking arm.mode, which
    lags recent set_mode calls via the heartbeat cache.
    """
    try:
        switch_mode(arm, mode=XArmMode.POSITION, log_fn=log_fn)
        move_to_configuration(arm, Lite6JointConfiguration.REST, log_fn=log_fn)
    except Exception as exc:
        log_fn(f"warning: move-to-{Lite6JointConfiguration.REST.name} during unprime failed: {exc}")
    arm.set_state(state=XArmState.STOP)
