"""
Standalone hardware experiments against the Ufactory Lite6 over
``xarm-python-sdk``. Each experiment is one top-level function; the
typer CLI at the bottom picks which one to run.

Goal: characterise the SDK surface (return codes, timings, quirks)
before wiring it into the aegis hardware backends. Findings get
logged to ``xarm_api.md`` at the repo root as we go.

Installed as the ``lite6_cli`` console script (see
``pyproject.toml``); run from any shell on a workstation that can
reach the arm:

    lite6_cli stream --ip 192.168.1.178

``-h`` works at every level:

    lite6_cli -h
    lite6_cli stream -h
"""

from __future__ import annotations

import contextlib
import io
import time
from typing import Annotated, Optional

import numpy as np
import typer

# The xarm SDK prints ``SDK_VERSION: <ver>`` to stdout on import with
# no off switch -- redirect stdout while we pull it in so the script's
# own output isn't preceded by that banner.
with contextlib.redirect_stdout(io.StringIO()):
    from xarm.wrapper import XArmAPI

# Default IP that ships from the factory + the deprecated codebase
# used. Override with --ip per invocation.
DEFAULT_IP = "192.168.1.178"

# Lite6 has 6 actuated arm joints. The xarm SDK pads joint vectors to
# 7 elements regardless of model; we slice down to this many.
LITE6_DOF = 6

# xarm SDK mode constants (see ``xarm/wrapper/xarm_api.py`` and
# ``manor.manipulators.lite6.driver`` for the manor side).
_XARM_MODE_POSITION = 0  # motion-plan position (set_servo_angle, set_position)
_XARM_MODE_SERVO_POSITION = 1  # low-latency joint streaming (set_servo_angle_j)
_XARM_MODE_VELOCITY = 4  # joint velocity (vc_set_joint_velocity)

# xarm SDK state constants:
#   0 - READY
#   4 - STOP
_XARM_STATE_READY = 0
_XARM_STATE_STOP = 4

# Default streaming rate for the joint-state dump experiment. Slow
# enough that the terminal can keep up; bump per-invocation if you're
# scope-watching a fast motion.
DEFAULT_STREAM_HZ = 20.0

# Per-call cap on joint-target deltas for the one-shot servo-j
# experiment. Mode 1 has no trajectory generation, so a big delta
# executes at firmware maxvel; the cap keeps a typo from flinging
# the arm. Override with ``--max_delta`` for deliberate larger moves.
DEFAULT_MAX_JOINT_DELTA_RAD = 0.1

# After ``set_servo_angle_j`` the call returns immediately; poll
# ``get_joint_states`` until the pose lands within this tolerance,
# or until ``_SETTLE_TIMEOUT_S`` elapses, before unpriming.
_SETTLE_TOLERANCE_RAD = 5e-3
_SETTLE_TIMEOUT_S = 5.0
_SETTLE_POLL_INTERVAL_S = 0.02


def _check(ret_code: int | tuple, op: str) -> None:
    """
    Raise a RuntimeError if an xarm SDK call returned a non-zero
    status code. Some calls return a plain int, others return a
    tuple whose first element is the code; handle both.
    """
    code = ret_code[0] if isinstance(ret_code, tuple) else ret_code
    if code != 0:
        raise RuntimeError(f"xarm SDK call {op!r} failed (code={code})")


def prime(arm: XArmAPI, mode: int = _XARM_MODE_SERVO_POSITION) -> None:
    """
    Bring the arm into a state where reads + writes work. Sequence
    mirrors ``Lite6Driver.prime()`` so the standalone script and the
    production driver verify the same activation path:

    1. ``clean_error()``      -- wipe any latched fault
    2. ``motion_enable(True)`` -- turn motors on (audible click)
    3. ``set_mode(<mode>)``   -- pick the control surface to use
    4. ``set_state(0)``       -- READY; required before motion calls

    Default ``mode`` is servo-position (1) so the existing stream
    command keeps its old behaviour. Position-mode commands pass
    mode=0; velocity-mode commands pass mode=4.
    """
    _check(arm.clean_error(), "clean_error")
    _check(arm.motion_enable(enable=True), "motion_enable")
    _check(arm.set_mode(mode=mode), f"set_mode({mode})")
    _check(arm.set_state(state=_XARM_STATE_READY), "set_state(ready)")


def unprime(arm: XArmAPI) -> None:
    """
    Inverse of ``prime``: stop motion, disable motors, disconnect.
    Best-effort -- still calls ``disconnect()`` even if the stop /
    disable steps fail, so we don't leave a dangling TCP session.
    """
    try:
        arm.set_state(state=_XARM_STATE_STOP)
        arm.motion_enable(enable=False)
    finally:
        arm.disconnect()


def read_joint_state(arm: XArmAPI) -> tuple[np.ndarray, np.ndarray]:
    """
    Read positions + velocities in radians / radians-per-second.

    The SDK's ``get_joint_states`` returns a 7-element vector for
    each field regardless of arm DOF (the 7th slot is reserved for
    7-DOF models); slice to ``LITE6_DOF`` here.
    """
    code, raw = arm.get_joint_states(is_radian=True)
    if code != 0:
        raise RuntimeError(f"get_joint_states failed (code={code})")
    positions, velocities, _torques = raw
    return (
        np.asarray(positions, dtype=np.float64)[:LITE6_DOF],
        np.asarray(velocities, dtype=np.float64)[:LITE6_DOF],
    )


# Properties + zero-arg methods to interrogate in ``probe_limits``.
# Anything that doesn't exist on the connected SDK falls back to
# ``<no such attr>`` rather than crashing the probe -- the point of
# this experiment is to discover what's exposed without prior knowledge.
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
    Read ``arm.<name>`` defensively. Some properties may not exist
    on the installed SDK version, or may raise when the arm is in
    a particular state -- the probe wants to print *something* for
    every entry rather than abort.
    """
    try:
        return getattr(arm, name)
    except AttributeError:
        return "<no such attr>"
    except Exception as e:
        return f"<error: {type(e).__name__}: {e}>"


def probe(arm: XArmAPI) -> None:
    """
    Print every property the SDK exposes for the connected arm:
    identity, live state, pose, calibration, limits, and motor
    state. Pure read -- the caller doesn't ``prime`` first so motors
    stay disabled. Any property that returns a sentinel or odd value
    is itself a useful finding worth logging into ``xarm_api.md``.
    """
    for name in _PROBE_PROPERTIES:
        typer.echo(f"  {name:<28} = {_safe_get(arm, name)!r}")


def send_joint_positions(
    arm: XArmAPI,
    targets: list[Optional[float]],
    max_delta_rad: float,
) -> None:
    """
    Move the arm to ``targets`` via mode 1 ``set_servo_angle_j`` --
    the same call ``Lite6Driver.write_joint_positions`` uses, so this
    experiment exercises the production code path. Any ``None`` entry
    in ``targets`` is replaced with the joint's current angle, so
    ``-j6 0.5`` wiggles joint 6 in isolation.

    Mode 1 has no trajectory generation: a single call sends the
    target straight to the servo loop and the arm rushes there as
    fast as the firmware joint maxvel allows. ``max_delta_rad`` is
    the cap on how big a per-call delta this script will accept --
    the production driver streams tiny per-tick deltas and bypasses
    this, but a one-shot CLI move with a 1 rad delta would be
    aggressive. Keep this small (~0.1 rad) unless you've cleared the
    workspace and know what you're doing.

    ``set_servo_angle_j`` returns immediately, so we poll
    ``get_joint_states`` afterwards until the pose lands within
    ``_SETTLE_TOLERANCE_RAD`` (or ``_SETTLE_TIMEOUT_S`` elapses) to
    avoid letting ``unprime`` hard-stop a moving arm.
    """
    current, _ = read_joint_state(arm)
    resolved = [c if t is None else t for t, c in zip(targets, current, strict=True)]
    delta = max(abs(r - c) for r, c in zip(resolved, current, strict=True))
    typer.echo(f"  current:   {[f'{v:+0.4f}' for v in current]}")
    typer.echo(f"  target:    {[f'{v:+0.4f}' for v in resolved]}")
    typer.echo(f"  max delta: {delta:.4f} rad (cap {max_delta_rad:.4f})")
    if delta > max_delta_rad:
        raise RuntimeError(
            f"requested move has max joint delta {delta:.4f} rad > "
            f"--max_delta {max_delta_rad:.4f} rad; raise --max_delta if "
            f"you want a bigger one-shot move (mode 1 has no trajectory "
            f"generation, so this would execute at firmware maxvel)"
        )
    _check(arm.set_servo_angle_j(angles=resolved, is_radian=True), "set_servo_angle_j")
    _wait_for_pose(arm, target=resolved)


def _wait_for_pose(arm: XArmAPI, target: list[float]) -> None:
    """
    Block until ``arm.angles`` is within ``_SETTLE_TOLERANCE_RAD`` of
    ``target`` on every joint, or ``_SETTLE_TIMEOUT_S`` elapses.
    Mode 1 ``set_servo_angle_j`` doesn't expose a ``wait`` semantic;
    this is the script's stand-in so unprime doesn't fight a moving arm.
    """
    deadline = time.monotonic() + _SETTLE_TIMEOUT_S
    target_arr = np.asarray(target, dtype=np.float64)
    while time.monotonic() < deadline:
        current, _ = read_joint_state(arm)
        if np.max(np.abs(current - target_arr)) <= _SETTLE_TOLERANCE_RAD:
            return
        time.sleep(_SETTLE_POLL_INTERVAL_S)
    typer.echo(f"  warning: pose did not settle within {_SETTLE_TIMEOUT_S:.1f}s")


def send_joint_velocities(arm: XArmAPI, velocities: list[float], duration_s: float) -> None:
    """
    Apply ``velocities`` (rad/s, length 6) for ``duration_s`` seconds,
    then zero them. Mode 4 must already be active. The zero-out is in
    a ``finally`` so an early Ctrl-C / exception still parks the arm
    instead of leaving the velocity command latched.
    """
    typer.echo(f"  velocities: {[f'{v:+0.4f}' for v in velocities]} for {duration_s:.3f} s")
    try:
        _check(
            arm.vc_set_joint_velocity(speeds=velocities, is_radian=True, duration=0),
            "vc_set_joint_velocity",
        )
        time.sleep(duration_s)
    finally:
        _check(
            arm.vc_set_joint_velocity(speeds=[0.0] * LITE6_DOF, is_radian=True, duration=0),
            "vc_set_joint_velocity(zero)",
        )


def stream_joint_state(arm: XArmAPI, hz: float, duration_s: Optional[float]) -> None:
    """
    Print joint positions + velocities at ``hz`` Hz until either
    ``duration_s`` elapses (if given) or Ctrl-C interrupts.
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


# Shared option spec so every command spells ``--ip`` the same way.
_IP_OPTION = typer.Option("--ip", help="Lite6 robot IP address.")


@app.callback()
def _main() -> None:
    """
    Top-level callback so typer treats this as a multi-command app
    even when only one ``@app.command`` is declared. Without it,
    typer hoists the lone command's args to the top level (e.g.
    ``lite6_cli --ip ...`` instead of ``lite6_cli stream --ip
    ...``), which would break invocations once a second experiment
    lands.
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
    Connect to the arm, prime it, and continuously print joint
    positions + velocities. The arm is unprimed on exit (Ctrl-C,
    duration elapsed, or unhandled exception) so motors are released
    and the TCP session is closed.
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
    Connect to the arm and dump every property the SDK exposes:
    identity, live state, pose, calibration, limits, motor state.
    Doesn't prime -- pure read; motors stay disabled. Anything
    printed as ``<no such attr>`` or ``<error: ...>`` is itself a
    finding worth recording in ``xarm_api.md``.
    """
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    try:
        probe(arm)
    finally:
        arm.disconnect()
        typer.echo("disconnected.")


@app.command("send_joint_positions")
def cmd_send_joint_positions(
    j1: Annotated[Optional[float], typer.Option("-j1", "--j1", help="Joint 1 target (rad). Default: current.")] = None,
    j2: Annotated[Optional[float], typer.Option("-j2", "--j2", help="Joint 2 target (rad). Default: current.")] = None,
    j3: Annotated[Optional[float], typer.Option("-j3", "--j3", help="Joint 3 target (rad). Default: current.")] = None,
    j4: Annotated[Optional[float], typer.Option("-j4", "--j4", help="Joint 4 target (rad). Default: current.")] = None,
    j5: Annotated[Optional[float], typer.Option("-j5", "--j5", help="Joint 5 target (rad). Default: current.")] = None,
    j6: Annotated[Optional[float], typer.Option("-j6", "--j6", help="Joint 6 target (rad). Default: current.")] = None,
    max_delta: Annotated[
        float,
        typer.Option(
            "--max_delta",
            help=(
                "Cap on the largest single-joint delta this command "
                "will accept (rad). Mode 1 has no trajectory "
                "generation, so big deltas execute at firmware maxvel."
            ),
        ),
    ] = DEFAULT_MAX_JOINT_DELTA_RAD,
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Move the arm to a joint pose using mode 1 ``set_servo_angle_j``
    -- the same SDK call ``Lite6Driver.write_joint_positions`` uses,
    so this exercises the production code path. Any joint not
    specified on the command line stays at its current angle, so
    ``-j6 0.5`` wiggles joint 6 in isolation.

    The largest commanded delta is capped to ``--max_delta``
    (default 0.1 rad ≈ 5.7°) because mode 1 has no trajectory
    generation -- a 1 rad delta would execute at firmware joint
    maxvel. Override for deliberately larger one-shot moves.

    After the call we poll the joints until the pose settles, since
    ``set_servo_angle_j`` returns immediately and ``unprime`` would
    otherwise hard-stop a still-moving arm.
    """
    targets: list[Optional[float]] = [j1, j2, j3, j4, j5, j6]
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    try:
        typer.echo("priming (mode 1, servo position)...")
        prime(arm, mode=_XARM_MODE_SERVO_POSITION)
        typer.echo("primed.")
        send_joint_positions(arm, targets=targets, max_delta_rad=max_delta)
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


@app.command("send_joint_velocities")
def cmd_send_joint_velocities(
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
    Apply the given joint velocity vector for ``--duration`` seconds,
    then zero the velocity. Uses mode 4 (joint velocity). Unspecified
    joints default to zero -- so ``-j6 0.3 -d 0.5`` wiggles joint 6
    only, all others held still.
    """
    velocities = [j1, j2, j3, j4, j5, j6]
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    try:
        typer.echo("priming (mode 4, velocity)...")
        prime(arm, mode=_XARM_MODE_VELOCITY)
        typer.echo("primed.")
        send_joint_velocities(arm, velocities=velocities, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


# typer apps are click apps under the hood; expose the click
# entry-point as ``cli`` so pyproject.toml's ``[project.scripts]``
# can wire ``lite6_cli`` to it directly (mirrors aegis_cli's pattern).
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
