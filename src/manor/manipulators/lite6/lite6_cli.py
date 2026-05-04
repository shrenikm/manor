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
from typing import Annotated, Optional

import numpy as np
import typer

# The xarm SDK prints SDK_VERSION: <ver> to stdout on import with no off switch -- redirect stdout while we
# pull it in so the script's own output isn't preceded by that banner.
with contextlib.redirect_stdout(io.StringIO()):
    from xarm.wrapper import XArmAPI

from manor.manipulators.lite6.xarm_helpers import (
    XArmMode,
    XArmState,
    check_xarm_call,
    connect,
    prime,
    switch_mode,
    unprime,
)

# Default IP that ships from the factory + the deprecated codebase used. Override with --ip per invocation.
DEFAULT_IP = "192.168.1.178"

# Lite6 has 6 actuated arm joints. The xarm SDK pads joint vectors to 7 elements regardless of model; we
# slice down to this many.
LITE6_DOF = 6

# Default streaming rate for the joint-state dump experiment. Slow enough that the terminal can keep up;
# bump per-invocation if you're scope-watching a fast motion.
DEFAULT_STREAM_HZ = 20.0

# Convergence tolerance for send_joint_positions: max absolute joint-angle error between arm.angles
# (heartbeat-cached measured pose) and the commanded target before we exit the convergence-poll loop.
_SETTLE_TOLERANCE_RAD = 5e-3

# Polling period for the convergence-poll loop in send_joint_positions. arm.angles updates at the
# heartbeat rate (~5 Hz), so anything finer than ~50 ms is wasted work.
_CONVERGE_POLL_PERIOD_S = 0.05


def _cli_log(message: str) -> None:
    """
    log_fn adapter for the cli: indented typer.echo so helper output blends with the surrounding cli
    progress lines.
    """
    typer.echo(f"  {message}")


# After motion_enable(True) the brakes release and the servos lock onto current encoder readings; this
# takes ~2 s of micro-motion to settle (observed empirically). The cli's manual command also needs this
# pause before issuing further state changes; the helpers module owns it for the prime path, but manual
# bypasses prime (no auto-move to PRIME) so we redeclare it here to keep the manual sequence in one place.
_MOTION_ENABLE_SETTLE_S = 2.0


def read_joint_state(arm: XArmAPI) -> tuple[np.ndarray, np.ndarray]:
    """
    Read positions + velocities in radians / radians-per-second.

    The SDK's get_joint_states returns a 7-element vector for each field regardless of arm DOF (the 7th
    slot is reserved for 7-DOF models); slice to LITE6_DOF here.
    """
    code, raw = arm.get_joint_states(is_radian=True)
    check_xarm_call(code, "get_joint_states", arm=arm)
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
    check_xarm_call(arm.set_servo_angle_j(angles=resolved, is_radian=True), "set_servo_angle_j", arm=arm)
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
        check_xarm_call(
            arm.vc_set_joint_velocity(speeds=velocities, is_radian=True, duration=0),
            "vc_set_joint_velocity",
            arm=arm,
        )
        time.sleep(duration_s)
    finally:
        check_xarm_call(
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
    no_args_is_help=True,
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
        prime(arm, log_fn=_cli_log)
        typer.echo("primed.")
        stream_joint_state(arm, hz=hz, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(arm, log_fn=_cli_log)
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


@app.command("connect")
def cmd_connect(
    ip: Annotated[str, _IP_OPTION] = DEFAULT_IP,
) -> None:
    """
    Explicit bring-up: clean_warn + clean_error, motion_enable(True), settle, set_mode(0),
    set_state(READY), and verify error_code/warn_code are clean (with a soft motion_enable-cycle recovery
    on failure). The arm is left energized in mode 0 / READY without being moved -- inverse of disconnect.

    Other commands (stream, send_jp, send_jv, manual) implicitly run this same sequence inside prime() at
    the start of every invocation; this dedicated command is for when you want the bring-up to be its own
    explicit step (e.g. after a power-cycle, or before opening a teach-pendant session, so the next motion
    command isn't slowed by the audible-click + 2 s encoder-relock that motion_enable triggers).
    """
    typer.echo(f"connecting to {ip}...")
    arm = XArmAPI(port=ip, is_radian=True)
    typer.echo("running connect sequence...")
    connect(arm, log_fn=_cli_log)
    typer.echo("connected (motors energized, mode 0, READY).")


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
    check_xarm_call(arm.clean_warn(), "clean_warn", arm=arm)
    check_xarm_call(arm.clean_error(), "clean_error", arm=arm)
    check_xarm_call(arm.motion_enable(enable=True), "motion_enable", arm=arm)
    time.sleep(_MOTION_ENABLE_SETTLE_S)
    switch_mode(arm, mode=XArmMode.MANUAL, log_fn=_cli_log)
    typer.echo("manual mode active. drag the arm freely. press Ctrl-C when done.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        typer.echo("\nexiting manual mode...")
    finally:
        typer.echo("unpriming...")
        unprime(arm, log_fn=_cli_log)
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
        prime(arm, mode=XArmMode.SERVO_POSITION, log_fn=_cli_log)
        typer.echo("primed.")
        send_joint_positions(arm, targets=targets)
    finally:
        typer.echo("unpriming...")
        unprime(arm, log_fn=_cli_log)
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
        prime(arm, mode=XArmMode.VELOCITY, log_fn=_cli_log)
        typer.echo("primed.")
        send_joint_velocities(arm, velocities=velocities, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(arm, log_fn=_cli_log)
        typer.echo("done.")


# typer apps are click apps under the hood; expose the click entry-point as cli so pyproject.toml's
# [project.scripts] can wire lite6_cli to it directly (mirrors aegis_cli's pattern).
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
