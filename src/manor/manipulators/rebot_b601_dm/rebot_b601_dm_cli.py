"""
Standalone hardware experiments against the Seeed reBot B601 DM over the motorbridge SDK. Each experiment is
one top-level function; the typer CLI at the bottom picks which one to run.

Goal: characterise the SDK surface (modes, feedback rates, quirks) before and while wiring it into the aegis
hardware backends. The driver and this CLI share the sequences in motorbridge_utils, so anything validated
here is exactly what production runs.

Installed as the rebot_b601_dm console script (see pyproject.toml); run from any shell on a workstation
with the arm's serial bridge attached:

    rebot_b601_dm stream --channel /dev/ttyACM0

-h works at every level:

    rebot_b601_dm -h
    rebot_b601_dm stream -h

SAFETY NOTES

* Large parts of this arm (including the gripper linkage) are 3D printed; full motor torque breaks them.
Every gripper command in this CLI goes through FORCE_POS with a torque ratio capped at
REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX, and every arm move streams torque-bounded MIT commands. Do not
bypass these with raw motorbridge calls unless you enjoy reprinting parts. The one exception is send_jv
(firmware VEL mode, integrator winds up on contact) -- keep its path clear.
* The all-zero pose (REST) is the vendor home: arm horizontal / sit-down, gripper closed. Joints 2 and 3
sit at their limit there. Motor zero offsets are volatile per session on this arm -- if positions look wrong
at connect, run the zero command with the arm physically held at the home pose.
"""

from __future__ import annotations

import time
from typing import Annotated, Optional

import numpy as np
import typer
from motorbridge import Mode

from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.model import REBOT_B601_DM_ARM_DOF
from manor.manipulators.rebot_b601_dm.motorbridge_utils import (
    REBOT_B601_DM_DEFAULT_CHANNEL,
    REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX,
    REBOT_B601_DM_MOTOR_SPECS,
    RebotB601DmBus,
    gripper_motor_rad_to_width,
    gripper_width_to_motor_rad,
    prime,
    unprime,
)

# Default streaming rate for the joint-state dump experiment. Slow enough that the terminal can keep up.
DEFAULT_STREAM_HZ = 20.0

# Default FORCE_POS torque ratio for CLI gripper commands: the vendor LeRobot integration's grip strength.
DEFAULT_GRIPPER_TORQUE_RATIO = 0.07

# How long send_jv applies the velocity before zeroing it if the operator gives no duration.
_DEFAULT_JV_DURATION_S = 1.0

# How far from REST the arm may be for disconnect to disable without asking. Beyond this the backdrivable,
# brakeless DM joints will fall under gravity when torque drops.
_DISCONNECT_REST_TOLERANCE_RAD = 0.2


def _cli_log(message: str) -> None:
    """
    log_fn adapter for the cli: indented typer.echo so helper output blends with the surrounding cli progress
    lines.
    """
    typer.echo(f"  {message}")


def stream_joint_state(bus: RebotB601DmBus, hz: float, duration_s: Optional[float]) -> None:
    """
    Print motor positions + velocities + torques at hz Hz until either duration_s elapses (if given) or
    Ctrl-C interrupts. Includes the gripper motor as the seventh column.
    """
    period = 1.0 / hz
    deadline = None if duration_s is None else time.monotonic() + duration_s
    typer.echo(f"streaming at {hz:.1f} Hz (Ctrl-C to stop)...")
    typer.echo(f"  {'q (rad)':<63} {'qdot (rad/s)':<63} tau (N*m)")
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            positions, velocities, torques = bus.read_state()
            q_str = " ".join(f"{x:+0.4f}" for x in positions)
            qdot_str = " ".join(f"{x:+0.4f}" for x in velocities)
            tau_str = " ".join(f"{x:+0.4f}" for x in torques)
            typer.echo(f"  {q_str:<63} {qdot_str:<63} {tau_str}")
            time.sleep(period)
    except KeyboardInterrupt:
        typer.echo("\ninterrupted.")


def probe(bus: RebotB601DmBus) -> None:
    """
    Dump identity + live state for every motor on the bus: spec (IDs, model), position, velocity, torque,
    and status code. Pure read -- motors stay disabled.
    """
    positions, velocities, torques = bus.read_state()
    typer.echo(f"  {'name':<10} {'send/recv':<12} {'model':<7} {'pos (rad)':<12} {'vel':<10} {'tau':<10} status")
    for i, spec in enumerate(REBOT_B601_DM_MOTOR_SPECS):
        state = bus._motor(spec.name).get_state()
        status = state.status_code if state is not None else "<never reported>"
        typer.echo(
            f"  {spec.name:<10} 0x{spec.send_id:02x}/0x{spec.feedback_id:02x}   {spec.model:<7} "
            f"{positions[i]:<+12.4f} {velocities[i]:<+10.4f} {torques[i]:<+10.4f} {status}"
        )


# --- CLI --------------------------------------------------------------------

app = typer.Typer(
    add_completion=False,
    help="Standalone hardware experiments against the reBot B601 DM over motorbridge.",
    no_args_is_help=True,
    context_settings={"help_option_names": ["-h", "--help"]},
)


# Shared option spec so every command spells --channel the same way.
_CHANNEL_OPTION = typer.Option("--channel", help="Serial bridge device for the arm's CAN bus.")

# Shared option spec for the gripper torque ratio: never accepted above the module ceiling.
_TORQUE_RATIO_OPTION = typer.Option(
    "--torque-ratio",
    min=0.01,
    max=REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX,
    help="FORCE_POS torque ceiling as a fraction of max motor torque. Vendor grips at 0.07.",
)


@app.callback()
def _main() -> None:
    """
    Top-level callback so typer treats this as a multi-command app even when only one @app.command is
    declared (mirrors lite6_cli).
    """


@app.command("probe")
def cmd_probe(
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Connect to the bus and dump identity + live state for every motor. Doesn't enable anything -- pure
    read; motors stay disabled (or in whatever state the previous session left them).
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    probe(bus)


@app.command("stream")
def cmd_stream(
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
    hz: Annotated[float, typer.Option("--hz", help="Print frequency in Hz.")] = DEFAULT_STREAM_HZ,
    duration: Annotated[
        Optional[float],
        typer.Option("--duration", help="How long to stream in seconds. Omit to run until Ctrl-C."),
    ] = None,
    passive: Annotated[
        bool,
        typer.Option(
            "--passive",
            help="Stream without priming: motors stay disabled so the arm can be moved by hand.",
        ),
    ] = False,
) -> None:
    """
    Continuously print motor positions + velocities + torques. By default the arm is primed first (motors
    enabled, moved to PRIME) and unprimed on exit; with --passive the motors are left disabled so the
    operator can move the arm by hand and watch the readouts (useful for verifying zero offsets).
    """
    bus = RebotB601DmBus(channel=channel)
    if passive:
        typer.echo(f"opening {channel} (passive; motors stay disabled)...")
        bus.connect(log_fn=_cli_log)
        stream_joint_state(bus, hz=hz, duration_s=duration)
        return
    typer.echo(f"opening {channel} and priming...")
    try:
        prime(bus, gripper_torque_ratio=DEFAULT_GRIPPER_TORQUE_RATIO, log_fn=_cli_log)
        typer.echo("primed.")
        stream_joint_state(bus, hz=hz, duration_s=duration)
    finally:
        typer.echo("unpriming...")
        unprime(bus, gripper_torque_ratio=DEFAULT_GRIPPER_TORQUE_RATIO, log_fn=_cli_log)
        typer.echo("done.")


@app.command("connect")
def cmd_connect(
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Explicit bring-up without motion: open the bus, clear latched motor errors, enable the motors, and
    configure modes (arm MIT holding the current pose at the streaming gains, gripper FORCE_POS). No move
    to PRIME. Inverse of disconnect.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    bus.clear_errors(log_fn=_cli_log)
    bus.enable_all(log_fn=_cli_log)
    bus.set_arm_mode(Mode.MIT, log_fn=_cli_log)
    positions, _ = bus.read_arm_state()
    bus.send_arm_mit(positions)
    bus.set_gripper_mode_force_pos(log_fn=_cli_log)
    typer.echo("connected (motors energized, arm holding in MIT, gripper FORCE_POS).")


@app.command("disconnect")
def cmd_disconnect(
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Full teardown: disable every motor and release the serial bridge. Run this when you're done with a
    session -- the DM motors hold torque while enabled, which warms them over time.

    The DM motors are backdrivable and have no brakes, so THE ARM FALLS when torque drops unless it is
    parked at REST (where gravity pushes the folded arm into its joint stops). If the arm is away from
    REST this command asks for confirmation and suggests running rest instead.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    positions, _ = bus.read_arm_state()
    rest = RebotB601DmJointConfiguration.REST.get_joint_positions_vector()
    max_error = float(np.max(np.abs(positions - rest)))
    if max_error > _DISCONNECT_REST_TOLERANCE_RAD:
        typer.echo(
            f"WARNING: arm is {max_error:.2f} rad away from REST. The DM motors have no brakes -- the arm "
            f"WILL FALL when disabled. Run 'rebot_b601_dm rest' to park it safely first."
        )
        typer.confirm("Disable anyway (support the arm!)?", abort=True)
    bus.disable_all(log_fn=_cli_log)
    bus.disconnect()
    typer.echo("disconnected (motors disabled, bridge released).")


@app.command("rest")
def cmd_rest(
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Move the arm to the REST joint configuration (the vendor home pose) and disable the motors. Mirrors
    unprime's shape -- the one command to safely park a running arm.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    bus.clear_errors(log_fn=_cli_log)
    bus.enable_all(log_fn=_cli_log)
    bus.set_gripper_mode_force_pos(log_fn=_cli_log)
    unprime(bus, gripper_torque_ratio=DEFAULT_GRIPPER_TORQUE_RATIO, log_fn=_cli_log)
    typer.echo(f"at {RebotB601DmJointConfiguration.REST.name} (motors disabled).")


@app.command("limp")
def cmd_limp(
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Disable motor torque so the arm can be moved freely by hand. Support the arm before running this --
    the joints will fall under gravity once torque drops. Ctrl-C exits; the motors stay disabled.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    typer.echo("SUPPORT THE ARM -- disabling torque in 2 seconds...")
    time.sleep(2.0)
    bus.disable_all(log_fn=_cli_log)
    typer.echo("torque disabled. move the arm freely; Ctrl-C to exit (motors stay disabled).")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        typer.echo("\ndone (motors still disabled).")


@app.command("zero")
def cmd_zero(
    joint: Annotated[
        Optional[list[str]],
        typer.Option(
            "--joint",
            help="Zero only these motors (repeatable, e.g. --joint gripper). Default: all seven.",
        ),
    ] = None,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Set the current physical pose as the zero position of every motor (or only --joint motors, e.g. a
    gripper-only re-zero after the linkage is reassembled). Physically hold the target joints at their
    zero pose before confirming: the vendor home pose for the arm (shoulder and forearm horizontal,
    gripper pointing forward), fully closed for the gripper -- this is the pose the URDF, the joint
    configurations, and the vendor tooling all treat as q = 0. The WHOLE bus is disabled first so the
    joints can be positioned by hand; the arm must be resting or supported.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    which = "every motor" if not joint else ", ".join(joint)
    typer.echo("Hold the target joints at their zero pose (home pose for the arm, fully closed gripper).")
    typer.confirm(f"Set the current pose as zero for {which}?", abort=True)
    bus.set_zero(joint_names=joint if joint else None, log_fn=_cli_log)
    typer.echo("zero set. verify with: rebot_b601_dm stream --passive")


@app.command("send_jp")
def cmd_send_jp(
    j1: Annotated[Optional[float], typer.Option("-j1", "--j1", help="Joint 1 target (rad). Default: current.")] = None,
    j2: Annotated[Optional[float], typer.Option("-j2", "--j2", help="Joint 2 target (rad). Default: current.")] = None,
    j3: Annotated[Optional[float], typer.Option("-j3", "--j3", help="Joint 3 target (rad). Default: current.")] = None,
    j4: Annotated[Optional[float], typer.Option("-j4", "--j4", help="Joint 4 target (rad). Default: current.")] = None,
    j5: Annotated[Optional[float], typer.Option("-j5", "--j5", help="Joint 5 target (rad). Default: current.")] = None,
    j6: Annotated[Optional[float], typer.Option("-j6", "--j6", help="Joint 6 target (rad). Default: current.")] = None,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Move the arm to a joint pose via the torque-bounded interpolated MIT move (the same helper prime /
    unprime use). Any joint not specified stays at its current angle, so -j6 0.5 wiggles joint 6 in
    isolation. The arm is primed first and left energized at the target (run rest / disconnect to park).
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel} and priming...")
    prime(bus, gripper_torque_ratio=DEFAULT_GRIPPER_TORQUE_RATIO, log_fn=_cli_log)
    current, _ = bus.read_arm_state()
    targets = [j1, j2, j3, j4, j5, j6]
    resolved = np.array([c if t is None else t for t, c in zip(targets, current, strict=True)], dtype=np.float64)
    typer.echo(f"  target: {[f'{v:+0.4f}' for v in resolved]}")
    bus.move_arm_to(resolved, "commanded", log_fn=_cli_log)
    typer.echo("converged (motors energized, holding in MIT; run rest or disconnect to park).")


@app.command("send_jv")
def cmd_send_jv(
    duration: Annotated[
        float, typer.Option("-d", "--duration", help="Duration to apply velocity (seconds).")
    ] = _DEFAULT_JV_DURATION_S,
    j1: Annotated[float, typer.Option("-j1", "--j1", help="Joint 1 velocity (rad/s).")] = 0.0,
    j2: Annotated[float, typer.Option("-j2", "--j2", help="Joint 2 velocity (rad/s).")] = 0.0,
    j3: Annotated[float, typer.Option("-j3", "--j3", help="Joint 3 velocity (rad/s).")] = 0.0,
    j4: Annotated[float, typer.Option("-j4", "--j4", help="Joint 4 velocity (rad/s).")] = 0.0,
    j5: Annotated[float, typer.Option("-j5", "--j5", help="Joint 5 velocity (rad/s).")] = 0.0,
    j6: Annotated[float, typer.Option("-j6", "--j6", help="Joint 6 velocity (rad/s).")] = 0.0,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Apply the given joint velocity vector for --duration seconds, then zero it. Switches the arm to the
    firmware VEL mode, whose integral term winds up to full torque against an obstacle -- keep the path
    clear and durations short (the production driver streams velocities through torque-bounded MIT
    instead). Unspecified joints default to zero -- so -j6 0.3 -d 0.5 wiggles joint 6 only. The zero-out
    runs in a finally so an early Ctrl-C still stops the arm.
    """
    velocities = np.array([j1, j2, j3, j4, j5, j6], dtype=np.float64)
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel} and priming...")
    prime(bus, gripper_torque_ratio=DEFAULT_GRIPPER_TORQUE_RATIO, log_fn=_cli_log)
    bus.set_arm_mode(Mode.VEL, log_fn=_cli_log)
    typer.echo(f"  velocities: {[f'{v:+0.4f}' for v in velocities]} for {duration:.3f} s")
    try:
        bus.send_arm_vel(velocities)
        time.sleep(duration)
    finally:
        bus.send_arm_vel(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))
        typer.echo("velocity zeroed (motors energized in VEL mode; run rest or disconnect to park).")


@app.command("gripper")
def cmd_gripper(
    width: Annotated[
        float,
        typer.Option(
            "-w",
            "--width",
            min=0.0,
            max=0.143,
            help="Target jaw opening width in metres (0 = closed, 0.143 = fully open).",
        ),
    ],
    torque_ratio: Annotated[float, _TORQUE_RATIO_OPTION] = DEFAULT_GRIPPER_TORQUE_RATIO,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Command the gripper to a jaw width via FORCE_POS -- firmware position control with a torque ceiling,
    the only way this stack ever drives the gripper. A close command onto an object grips at the torque
    ceiling instead of crushing. Assumes the motors were already brought up (run connect first).
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    bus.enable_all(log_fn=_cli_log)
    bus.set_gripper_mode_force_pos(log_fn=_cli_log)
    motor_rad = gripper_width_to_motor_rad(width)
    typer.echo(f"  width {width:.4f} m -> motor {motor_rad:+.4f} rad at torque ratio {torque_ratio:.2f}")
    bus.send_gripper_force_pos(motor_rad=motor_rad, torque_ratio=torque_ratio)
    time.sleep(2.0)
    current_rad, _ = bus.read_gripper_state()
    typer.echo(f"  gripper at motor {current_rad:+.4f} rad (width {gripper_motor_rad_to_width(current_rad):.4f} m)")
    typer.echo("done (motors energized; run disconnect to park).")


# typer apps are click apps under the hood; expose the click entry-point as cli so pyproject.toml's
# [project.scripts] can wire rebot_b601_dm to it directly (mirrors lite6_cli's pattern).
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
