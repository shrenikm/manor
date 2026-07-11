"""
Standalone hardware experiments against the Seeed reBot B601 DM over the motorbridge SDK. Each experiment is
one top-level function; the typer CLI at the bottom picks which one to run.

Goal: characterise the SDK surface (modes, feedback rates, quirks) before and while wiring it into the aegis
hardware backends. The driver and this CLI share the sequences in motorbridge_helpers, so anything validated
here is exactly what production runs.

Installed as the rebot console script (see pyproject.toml); run from any shell on a workstation
with the arm's serial bridge attached:

    rebot stream --channel /dev/ttyACM0

-h works at every level:

    rebot -h
    rebot stream -h

SAFETY NOTES

* Large parts of this arm (including the gripper linkage) are 3D printed; full motor torque breaks them.
Every gripper command in this CLI goes through FORCE_POS with a torque ratio capped at
REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX. The arm is brought up ONCE with connect (the only command that
clears errors and enables motors); after that send_jp / send_jv are pure motion -- they open the bus and
stream a torque-bounded MIT trajectory (bounded at kp * clamp, no integrator) and touch nothing else, so
they can be chained back to back without disturbing the hold. rest parks at REST; disconnect / limp
disable. Do not bypass these with raw motorbridge calls unless you enjoy reprinting parts.
* The all-zero pose (REST) is the vendor home: arm horizontal / sit-down, gripper closed. Joints 2 and 3
sit at their limit there. Motor zero offsets are volatile per session on this arm -- if positions look wrong
at connect, run the zero command with the arm physically held at the home pose.
"""

from __future__ import annotations

import readline  # noqa: F401
import time
from typing import Annotated, Optional

import numpy as np
import typer
from motorbridge import Mode

from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.model import REBOT_B601_DM_ARM_DOF
from manor.manipulators.rebot_b601_dm.motorbridge_helpers import (
    REBOT_B601_DM_ARM_CONFIGURATION_MOVE_SPEED_RAD_S,
    REBOT_B601_DM_DEFAULT_CHANNEL,
    REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M,
    REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX,
    REBOT_B601_DM_MOTOR_SPECS,
    RebotB601DmArmStreamer,
    RebotB601DmBus,
    gripper_motor_rad_to_width,
    gripper_width_to_motor_rad,
)

# Default streaming rate for the joint-state dump experiment. Slow enough that the terminal can keep up.
DEFAULT_STREAM_HZ = 20.0

# Default FORCE_POS torque ratio for CLI gripper commands: the vendor LeRobot integration's grip strength.
DEFAULT_GRIPPER_TORQUE_RATIO = 0.07

# How long send_jv applies the velocity before zeroing it if the operator gives no duration.
_DEFAULT_JV_DURATION_S = 1.0

# Default advance speed for send_jp's MIT move (rad/s): the gentle bring-up speed, overridable per
# invocation with --max-speed. MIT has no firmware speed limit, so this is the rate the streamed target
# ramps toward the goal; the per-tick command-error clamp bounds the torque independently.
_DEFAULT_SEND_JP_SPEED_RAD_S = REBOT_B601_DM_ARM_CONFIGURATION_MOVE_SPEED_RAD_S
_MAX_SEND_JP_SPEED_RAD_S = 2.0

# Command-error clamp for send_jp's move (rad). The per-joint torque ceiling is about kp*clamp (kp=120 on
# j1-3, 18 on j4-6), so this is the knob for "a heavy joint can't overcome gravity + geartrain stiction to
# reach its target." Default matches the bus move default; the ceiling keeps j1-3 under the DM-J4340's
# ~28 N*m even at the max (120*0.25 = 30). Raise it on hardware until the joint moves cleanly, then we
# bake the working value into the bus default.
_DEFAULT_SEND_JP_ERROR_CLAMP_RAD = 0.08
_MAX_SEND_JP_ERROR_CLAMP_RAD = 0.25

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


def _energize_and_hold(bus: RebotB601DmBus) -> None:
    """
    Bring the motors up and hold the CURRENT pose -- the reBot's only bring-up. Enables the motors, puts
    the arm in MIT holding its measured pose (bounded torque, no motion), and the gripper in FORCE_POS.
    Unlike the lite6 there is no reason to drive to a PRIME pose just to energize, and doing so would swing
    the big joints unexpectedly, so nothing in this cli moves the arm on its own -- moves are explicit
    (send_jp to a commanded pose, rest to park at REST).
    """
    bus.connect(log_fn=_cli_log)
    bus.clear_errors(log_fn=_cli_log)
    bus.enable_all(log_fn=_cli_log)
    bus.set_arm_mode(Mode.MIT, log_fn=_cli_log)
    positions, _ = bus.read_arm_state()
    bus.send_arm_mit(positions)
    bus.set_gripper_mode_force_pos(log_fn=_cli_log)


# Joint flags accepted in the send_jp REPL, mapped to their index in the arm vector.
_REPL_JOINT_FLAGS: dict[str, int] = {}
for _j in range(REBOT_B601_DM_ARM_DOF):
    _REPL_JOINT_FLAGS[f"-j{_j + 1}"] = _j
    _REPL_JOINT_FLAGS[f"--j{_j + 1}"] = _j

# How long the REPL waits for the arm to reach REST when parking on exit before it disables regardless.
_REPL_PARK_TIMEOUT_S = 8.0

_REPL_HELP = (
    "  -jN <rad> ...   retarget joint(s), e.g. '-j3 -0.3 -j4 0.0' (unset joints keep their target)\n"
    "  rest            set every joint target to 0\n"
    "  -s <rad/s>      set the ramp speed\n"
    "  -c <rad>        set the command-error clamp (torque ceiling ~ kp*clamp)\n"
    "  p / <enter>     print target vs measured\n"
    "  h / help        show this help\n"
    "  q / quit        park at REST, disable, and exit"
)


def _print_repl_status(streamer: RebotB601DmArmStreamer) -> None:
    target, measured, speed, clamp = streamer.snapshot()
    typer.echo("  target   " + " ".join(f"{v:+0.3f}" for v in target))
    typer.echo("  measured " + " ".join(f"{v:+0.3f}" for v in measured) + f"   (speed {speed:.2f}, clamp {clamp:.3f})")


def _apply_repl_command(line: str, streamer: RebotB601DmArmStreamer) -> None:
    """
    Parse one REPL line and push the change into the streamer. Joint flags update only the named joints of
    the current target (the rest hold); -s / -c adjust the ramp speed / command-error clamp live.
    """
    tokens = line.split()
    target, _measured, _speed, _clamp = streamer.snapshot()
    target = target.copy()
    i = 0
    while i < len(tokens):
        token = tokens[i]
        if i + 1 >= len(tokens):
            raise ValueError(f"'{token}' needs a value")
        value = float(tokens[i + 1])
        if token in _REPL_JOINT_FLAGS:
            target[_REPL_JOINT_FLAGS[token]] = value
        elif token in ("-s", "--speed"):
            streamer.set_speed(value)
        elif token in ("-c", "--clamp"):
            streamer.set_clamp(value)
        else:
            raise ValueError(f"unknown token '{token}'")
        i += 2
    streamer.set_target(target)


def _park_and_disable(bus: RebotB601DmBus, streamer: RebotB601DmArmStreamer) -> None:
    """
    Park at REST while the streamer is still holding the arm, then stop the stream and disable. Waiting is
    done off the streamer's shared snapshot so the main thread never touches the bus while the stream runs.
    """
    streamer.set_target(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))
    deadline = time.monotonic() + _REPL_PARK_TIMEOUT_S
    while time.monotonic() < deadline:
        _target, measured, _speed, _clamp = streamer.snapshot()
        if float(np.max(np.abs(measured))) < 0.05:
            break
        time.sleep(0.05)
    streamer.stop()
    bus.disable_all(log_fn=_cli_log)


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
) -> None:
    """
    Continuously print motor positions + velocities + torques. Pure read: it opens the bus and prints, and
    never enables, disables, or moves anything -- the motors stay in whatever state the last command left
    them (energized + holding after connect / send_jp, disabled after limp / disconnect). Use limp first if
    you want to hand-move the arm while watching the readouts.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    stream_joint_state(bus, hz=hz, duration_s=duration)


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
    _energize_and_hold(bus)
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
            f"WILL FALL when disabled. Run 'rebot rest' to park it safely first."
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
    Park the arm at REST (the vendor home pose) and disable the motors -- the one-shot command to bring a
    possibly-disabled arm down safely (e.g. after a session timed out and the motors dropped). It
    re-energizes first, then controls the descent to REST, then closes the gripper and disables. For live
    iterative work use the send_jp REPL, which parks on exit.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel} and bringing up to park...")
    _energize_and_hold(bus)
    rest_vector = RebotB601DmJointConfiguration.REST.get_joint_positions_vector()
    bus.move_arm_to(rest_vector, RebotB601DmJointConfiguration.REST.name, log_fn=_cli_log)
    bus.send_gripper_force_pos(motor_rad=0.0, torque_ratio=DEFAULT_GRIPPER_TORQUE_RATIO)
    bus.disable_all(log_fn=_cli_log)
    typer.echo(f"parked at {RebotB601DmJointConfiguration.REST.name} (motors disabled).")


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
    typer.echo("zero set (motors disabled). verify by hand-moving and watching: rebot stream")


@app.command("send_jp")
def cmd_send_jp(
    speed: Annotated[
        float,
        typer.Option(
            "-s",
            "--speed",
            min=0.05,
            max=_MAX_SEND_JP_SPEED_RAD_S,
            help="Ramp speed toward the target (rad/s). Adjustable live with '-s <val>'.",
        ),
    ] = _DEFAULT_SEND_JP_SPEED_RAD_S,
    clamp: Annotated[
        float,
        typer.Option(
            "-c",
            "--clamp",
            min=0.02,
            max=_MAX_SEND_JP_ERROR_CLAMP_RAD,
            help="Command-error clamp (rad); torque ceiling ~ kp*clamp. Adjustable live with '-c <val>'.",
        ),
    ] = _DEFAULT_SEND_JP_ERROR_CLAMP_RAD,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Interactive joint REPL. Brings the arm up once, then a background thread streams it continuously -- the
    arm always holds (the stream keeps the DM command-timeout from firing) and moves smoothly whenever you
    retarget a joint, exactly the way aegis streams the production driver. Type '-jN <rad>' to retarget
    joints (unset joints keep their target), 'rest' to zero every joint, 'q' to park at REST and exit.
    Holding the bus open for the whole session is why this works where one-shot commands drop the arm.
    """
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel} and bringing up...")
    _energize_and_hold(bus)
    streamer = RebotB601DmArmStreamer(bus, speed_rad_s=speed, error_clamp_rad=clamp)
    streamer.start()
    typer.echo("streaming (arm held). type '-jN <rad>' to move, 'rest' to zero, 'h' for help, 'q' to park+quit.")
    try:
        while True:
            try:
                line = input("jp> ").strip()
            except EOFError:
                break
            if line in ("q", "quit", "exit"):
                break
            if line in ("", "p", "?"):
                _print_repl_status(streamer)
                continue
            if line in ("h", "help"):
                typer.echo(_REPL_HELP)
                continue
            if line == "rest":
                streamer.set_target(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))
                _print_repl_status(streamer)
                continue
            try:
                _apply_repl_command(line, streamer)
            except (ValueError, IndexError) as exc:
                typer.echo(f"  ? {exc} (try 'h')")
                continue
            _print_repl_status(streamer)
    except KeyboardInterrupt:
        typer.echo("")
    finally:
        typer.echo("parking at REST and disabling...")
        _park_and_disable(bus, streamer)
        typer.echo("done (motors disabled).")


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
    clamp: Annotated[
        float,
        typer.Option(
            "-c",
            "--clamp",
            min=0.02,
            max=_MAX_SEND_JP_ERROR_CLAMP_RAD,
            help="Command-error clamp (rad); per-joint torque ceiling is about kp*clamp. Raise if a heavy "
            "joint can't move at the requested velocity.",
        ),
    ] = _DEFAULT_SEND_JP_ERROR_CLAMP_RAD,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Move each named joint at the given velocity for --duration seconds. PURE motion: opens the bus and
    streams a torque-bounded MIT trajectory whose target advances at the requested per-joint velocity, then
    holds -- it does NOT enable motors, clear errors, or change mode (run connect once first). This is a
    velocity via a ramping MIT position target, so the other joints are held in place and torque stays
    bounded, unlike the firmware VEL mode (whose integral term winds up on contact). Unspecified joints
    stay put -- so -j6 0.3 -d 0.5 moves joint 6 only, ending 0.15 rad along.
    """
    velocities = np.array([j1, j2, j3, j4, j5, j6], dtype=np.float64)
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel}...")
    bus.connect(log_fn=_cli_log)
    current, _ = bus.read_arm_state()
    endpoint = current + velocities * duration
    typer.echo(f"  velocities: {[f'{v:+0.4f}' for v in velocities]} for {duration:.3f} s, clamp {clamp:.3f} rad")
    bus.move_arm_to(endpoint, "velocity endpoint", speed_rad_s=velocities, error_clamp_rad=clamp, log_fn=_cli_log)


@app.command("gripper")
def cmd_gripper(
    width: Annotated[
        float,
        typer.Option(
            "-w",
            "--width",
            min=0.0,
            max=REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M,
            help=(
                "Target jaw opening width in metres "
                f"(0 = closed, {REBOT_B601_DM_GRIPPER_MEASURED_OPEN_WIDTH_M:.3f} = fully open)."
            ),
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
# [project.scripts] can wire rebot to it directly (mirrors lite6_cli's pattern).
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
