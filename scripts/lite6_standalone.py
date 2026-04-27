"""
Standalone hardware experiments against the Ufactory Lite6 over
``xarm-python-sdk``. Each experiment is one top-level function; the
typer CLI at the bottom picks which one to run.

Goal: characterise the SDK surface (return codes, timings, quirks)
before wiring it into the aegis hardware backends. Findings get
logged to ``xarm_api.md`` at the repo root as we go.

Run from a shell on a workstation that can reach the arm's IP:

    python scripts/lite6_standalone.py stream-angles --ip 192.168.1.178

``-h`` works at every level:

    python scripts/lite6_standalone.py -h
    python scripts/lite6_standalone.py stream-angles -h
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
# ``manor.manipulators.lite6.driver`` for the manor side):
#   0 - position (motion-plan style)
#   1 - servo position (low-latency joint streaming)
#   4 - joint velocity
_XARM_MODE_SERVO_POSITION = 1

# xarm SDK state constants:
#   0 - READY
#   4 - STOP
_XARM_STATE_READY = 0
_XARM_STATE_STOP = 4

# Default streaming rate for the joint-angle dump experiment. Slow
# enough that the terminal can keep up; bump per-invocation if you're
# scope-watching a fast motion.
DEFAULT_STREAM_HZ = 20.0


def _check(ret_code: int | tuple, op: str) -> None:
    """
    Raise a RuntimeError if an xarm SDK call returned a non-zero
    status code. Some calls return a plain int, others return a
    tuple whose first element is the code; handle both.
    """
    code = ret_code[0] if isinstance(ret_code, tuple) else ret_code
    if code != 0:
        raise RuntimeError(f"xarm SDK call {op!r} failed (code={code})")


def prime(arm: XArmAPI) -> None:
    """
    Bring the arm into a state where reads + writes work. Sequence
    mirrors ``Lite6Driver.prime()`` so the standalone script and the
    production driver verify the same activation path:

    1. ``clean_error()``      -- wipe any latched fault
    2. ``motion_enable(True)`` -- turn motors on (audible click)
    3. ``set_mode(1)``        -- servo position mode
    4. ``set_state(0)``       -- READY; required before motion calls
    """
    _check(arm.clean_error(), "clean_error")
    _check(arm.motion_enable(enable=True), "motion_enable")
    _check(arm.set_mode(mode=_XARM_MODE_SERVO_POSITION), "set_mode(servo_position)")
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


def stream_joint_angles(arm: XArmAPI, hz: float, duration_s: Optional[float]) -> None:
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


@app.command("stream-angles")
def cmd_stream_angles(
    ip: Annotated[str, typer.Option("--ip", help="Lite6 robot IP address.")] = DEFAULT_IP,
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
        stream_joint_angles(arm, hz=hz, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(arm)
        typer.echo("done.")


if __name__ == "__main__":
    app()
