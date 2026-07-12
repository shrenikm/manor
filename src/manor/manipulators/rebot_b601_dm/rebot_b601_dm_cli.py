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
Every gripper command goes through FORCE_POS with a torque ratio capped at
REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX, and every arm motion goes through the torque-bounded MIT law (the
commanded position is leashed within the command-error clamp of the measured pose, so torque stays
bounded at kp * clamp, no integrator). send_jp / send_jv are interactive REPLs that bring the arm up,
stream continuously so it never times out (the DM watchdog disables a motor once commands stop), and park
at REST on exit -- send_jp sets joint positions, send_jv sets joint velocities. float is a pure
gravity-compensation mode: it streams a scaled g(q) torque feedforward (from the Drake model) with NO
position hold (kp = 0) plus light damping, so the arm carries its own weight and is moved around by hand;
its scale is tuned live ('-g <scale>') until the arm hangs weightless, and it disables (goes limp) on exit,
so support the arm. connect / rest / disconnect / limp are the one-shot bring-up / park / teardown. Do not
bypass these with raw motorbridge calls unless you enjoy reprinting parts.
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

from manor.manipulators.rebot_b601_dm.gravity import RebotB601DmGravityModel
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

# Gravity-compensation feedforward scale. Starts at 0 everywhere so the sign is validated on hardware by
# ramping it up (live, with '-g <scale>'): the droop shrinks toward zero if g(q) has the right sign and
# grows if it is flipped, and the leashed position loop bounds the arm either way. A tuned working value
# (below 1, absorbing geartrain friction / model error) gets passed explicitly with --tau-scale once found.
_DEFAULT_TAU_SCALE = 0.0
_MAX_TAU_SCALE = 1.5

# Float-mode gains: NO position stiffness (kp = 0), so the arm holds no target and the scaled g(q)
# feedforward is the only thing carrying its weight -- that is exactly what makes the scale tunable by feel.
# kd is light velocity damping so the arm settles instead of drifting or twitching when released. Starting
# point; tune on hardware.
_FLOAT_MIT_KP: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_FLOAT_MIT_KD: tuple[float, ...] = (5.0, 5.0, 5.0, 1.0, 1.0, 1.0)


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


def _repl_help(velocity_mode: bool, gravity_enabled: bool) -> str:
    if velocity_mode:
        head = [
            "  -jN <rad/s> ... -d <s>   move joint(s) at velocity for <s> seconds then stop, e.g.",
            "                           '-j6 0.3 -d 1' -- a duration is REQUIRED (motion is never open-ended)",
        ]
    else:
        head = [
            "  -jN <rad> ...            retarget joint(s), e.g. '-j3 -0.3 -j4 0.0' (unset joints keep theirs)",
            "  -s <rad/s>               ramp speed",
        ]
    gravity_line = (
        ["  -g <scale>               gravity-comp feedforward scale (ramp up from 0 to engage)"]
        if gravity_enabled
        else []
    )
    return "\n".join(
        head
        + [
            "  stop                     freeze the arm where it is",
            "  rest                     ramp every joint home to 0",
            "  -c <rad>                 command-error clamp (torque ceiling ~ kp*clamp)",
        ]
        + gravity_line
        + [
            "  p / <enter>              print state",
            "  h / help                 show this help",
            "  q / quit                 park at REST, disable, and exit",
        ]
    )


def _print_repl_status(streamer: RebotB601DmArmStreamer) -> None:
    state = streamer.snapshot()
    if state.velocity_mode:
        typer.echo("  velocity " + " ".join(f"{v:+0.3f}" for v in state.velocity) + "  rad/s")
    else:
        typer.echo("  target   " + " ".join(f"{v:+0.3f}" for v in state.target))
    settings = f"speed {state.speed:.2f}, clamp {state.clamp:.3f}"
    if state.gravity_enabled:
        settings += f", grav {state.tau_scale:.2f}"
    typer.echo("  measured " + " ".join(f"{v:+0.3f}" for v in state.measured) + f"   ({settings})")


def _parse_repl_tokens(
    tokens: list[str],
) -> tuple[dict[int, float], float | None, float | None, float | None, float | None]:
    """
    Parse REPL '-jN val' / '-s val' / '-c val' / '-d val' / '-g val' pairs into
    (joint_values, speed, clamp, duration, tau_scale). A flag with no value, or an unrecognised token,
    raises ValueError.
    """
    joint_values: dict[int, float] = {}
    speed: float | None = None
    clamp: float | None = None
    duration: float | None = None
    tau_scale: float | None = None
    i = 0
    while i < len(tokens):
        token = tokens[i]
        if i + 1 >= len(tokens):
            raise ValueError(f"'{token}' needs a value")
        value = float(tokens[i + 1])
        if token in _REPL_JOINT_FLAGS:
            joint_values[_REPL_JOINT_FLAGS[token]] = value
        elif token in ("-s", "--speed"):
            speed = value
        elif token in ("-c", "--clamp"):
            clamp = value
        elif token in ("-d", "--duration"):
            duration = value
        elif token in ("-g", "--gravity-scale"):
            tau_scale = value
        else:
            raise ValueError(f"unknown token '{token}'")
        i += 2
    return joint_values, speed, clamp, duration, tau_scale


def _apply_position_command(line: str, streamer: RebotB601DmArmStreamer) -> None:
    """
    Position REPL line: retarget the named joints (others keep their target); -s / -c / -g adjust the ramp
    speed, command-error clamp, and gravity-comp scale live.
    """
    joint_values, speed, clamp, duration, tau_scale = _parse_repl_tokens(line.split())
    if duration is not None:
        raise ValueError("-d (duration) is only for the velocity repl")
    if speed is not None:
        streamer.set_speed(speed)
    if clamp is not None:
        streamer.set_clamp(clamp)
    if tau_scale is not None:
        streamer.set_tau_scale(tau_scale)
    if joint_values:
        target = streamer.snapshot().target.copy()
        for index, value in joint_values.items():
            target[index] = value
        streamer.set_target(target)


def _apply_velocity_pulse(line: str, streamer: RebotB601DmArmStreamer) -> None:
    """
    Velocity REPL line: move the named joints at their velocities for -d seconds, then freeze. A duration is
    REQUIRED so a joint can never run open-ended into a limit (or the cameras). Each pulse starts from zero,
    so only the named joints move; -c adjusts the clamp and -g the gravity-comp scale. The wait blocks the
    prompt while the arm streams; Ctrl-C ends the pulse early and freezes, without leaving the repl.
    """
    joint_values, speed, clamp, duration, tau_scale = _parse_repl_tokens(line.split())
    if speed is not None:
        raise ValueError("no -s in the velocity repl -- the -jN values are the velocities")
    if clamp is not None:
        streamer.set_clamp(clamp)
    if tau_scale is not None:
        streamer.set_tau_scale(tau_scale)
    if not joint_values:
        return
    if duration is None or duration <= 0.0:
        raise ValueError("velocity commands need a positive duration, e.g. '-j6 0.3 -d 1'")
    velocity = np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64)
    for index, value in joint_values.items():
        velocity[index] = value
    streamer.set_velocity(velocity)
    typer.echo(f"  moving {duration:.2f}s (Ctrl-C to stop early)...")
    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        typer.echo("  interrupted")
    finally:
        # Zero the velocity rather than snapping to a position hold. During the pulse the commanded
        # interpolant leads the arm by up to the clamp; a position hold would capture the (trailing)
        # measured pose and yank the joint back by that lead. Zeroing velocity lets the interpolant coast
        # down with the arm so it decelerates smoothly to where it was heading, with no jump back.
        streamer.set_velocity(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))


def _park_and_disable(bus: RebotB601DmBus, streamer: RebotB601DmArmStreamer) -> None:
    """
    Park at REST while the streamer is still holding the arm, then stop the stream and disable. Waiting is
    done off the streamer's shared snapshot so the main thread never touches the bus while the stream runs.
    """
    streamer.set_target(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))
    deadline = time.monotonic() + _REPL_PARK_TIMEOUT_S
    while time.monotonic() < deadline:
        if float(np.max(np.abs(streamer.snapshot().measured))) < 0.05:
            break
        time.sleep(0.05)
    streamer.stop()
    bus.disable_all(log_fn=_cli_log)


def _run_arm_repl(
    channel: str,
    speed: float,
    clamp: float,
    velocity_mode: bool,
    gravity: bool = False,
    tau_scale: float = _DEFAULT_TAU_SCALE,
    kp: np.ndarray | None = None,
    kd: np.ndarray | None = None,
) -> None:
    """
    Shared interactive REPL for send_jp (position), send_jv (velocity), and float (gravity-comp hold).
    Brings the arm up once, then a background streamer holds and moves it continuously so it never times
    out; each line retargets joints (position) or sets joint velocities (velocity). When gravity is set the
    streamer adds a g(q) feedforward scaled by tau_scale (live via '-g'), with kp / kd overriding the
    default move gains (float uses the soft streaming gains). Parks at REST and disables on exit.
    """
    gravity_model: RebotB601DmGravityModel | None = None
    if gravity:
        typer.echo("building gravity model (Drake)...")
        gravity_model = RebotB601DmGravityModel()
        gravity_model.warm()
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel} and bringing up...")
    _energize_and_hold(bus)
    streamer = RebotB601DmArmStreamer(
        bus,
        speed_rad_s=speed,
        error_clamp_rad=clamp,
        kp=kp,
        kd=kd,
        gravity_model=gravity_model,
        tau_scale=tau_scale,
    )
    streamer.start()
    prompt = "jv> " if velocity_mode else "jp> "
    move_hint = "'-jN <rad/s> -d <s>' to pulse a joint" if velocity_mode else "'-jN <rad>' to move a joint"
    typer.echo(f"streaming (arm held). {move_hint}, 'rest' home, 'stop' freeze, 'h' help, 'q' quit.")
    if gravity_model is not None:
        typer.echo(
            f"  gravity comp ON (scale {tau_scale:.2f}). SUPPORT THE ARM, then ramp up slowly with "
            f"'-g <scale>' toward 1.0: the droop should shrink -- if it sags harder the sign is wrong, "
            f"set '-g 0'."
        )
    try:
        while True:
            try:
                line = input(prompt).strip()
            except EOFError:
                break
            if line in ("q", "quit", "exit"):
                break
            if line in ("", "p", "?"):
                _print_repl_status(streamer)
                continue
            if line in ("h", "help"):
                typer.echo(_repl_help(velocity_mode, gravity_model is not None))
                continue
            if line == "rest":
                streamer.set_target(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))
                _print_repl_status(streamer)
                continue
            if line == "stop":
                streamer.hold()
                _print_repl_status(streamer)
                continue
            try:
                if velocity_mode:
                    _apply_velocity_pulse(line, streamer)
                else:
                    _apply_position_command(line, streamer)
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


def _print_float_status(streamer: RebotB601DmArmStreamer) -> None:
    state = streamer.snapshot()
    typer.echo(
        "  measured " + " ".join(f"{v:+0.3f}" for v in state.measured) + f"   (grav scale {state.tau_scale:.2f})"
    )


def _run_float_repl(channel: str, tau_scale: float) -> None:
    """
    Pure gravity-compensation float. Brings the arm up, then streams ONLY a scaled g(q) torque feedforward
    plus light velocity damping -- no position hold (kp = 0) -- so the arm carries its own weight and is
    moved around by hand as if it were floating. Tune the scale live with '-g <scale>': below the right
    value the arm sinks under its weight, above it drifts upward, and at it the arm hangs weightless wherever
    it is left. 'q' disables the motors (the arm goes limp, so support it) and exits.
    """
    typer.echo("building gravity model (Drake)...")
    gravity_model = RebotB601DmGravityModel()
    gravity_model.warm()
    bus = RebotB601DmBus(channel=channel)
    typer.echo(f"opening {channel} and bringing up...")
    _energize_and_hold(bus)
    streamer = RebotB601DmArmStreamer(
        bus,
        kp=np.asarray(_FLOAT_MIT_KP, dtype=np.float64),
        kd=np.asarray(_FLOAT_MIT_KD, dtype=np.float64),
        gravity_model=gravity_model,
        tau_scale=tau_scale,
    )
    streamer.start()
    typer.echo(
        f"FLOATING with gravity comp at scale {tau_scale:.2f}. SUPPORT THE ARM -- with no position hold it "
        f"is carried ONLY by the feedforward, so at scale 0 it is effectively limp."
    )
    typer.echo(
        "  '-g <scale>' to tune (arm sinks if low, rises if high, hangs weightless when right), 'p' state, "
        "'h' help, 'q' quit."
    )
    try:
        while True:
            try:
                line = input("float> ").strip()
            except EOFError:
                break
            if line in ("q", "quit", "exit"):
                break
            if line in ("", "p", "?"):
                _print_float_status(streamer)
                continue
            if line in ("h", "help"):
                typer.echo(
                    "  -g <scale>               set the gravity-comp feedforward scale (the tuning knob)\n"
                    "  p / <enter>              print measured pose + scale\n"
                    "  q / quit                 disable motors and exit (support the arm first)"
                )
                continue
            tokens = line.split()
            try:
                if len(tokens) == 2 and tokens[0] in ("-g", "--gravity-scale"):
                    streamer.set_tau_scale(float(tokens[1]))
                else:
                    raise ValueError(f"unknown command '{line}'")
            except ValueError as exc:
                typer.echo(f"  ? {exc} (try 'h')")
                continue
            _print_float_status(streamer)
    except KeyboardInterrupt:
        typer.echo("")
    finally:
        typer.echo("SUPPORT THE ARM -- stopping float and disabling.")
        streamer.stop()
        bus.disable_all(log_fn=_cli_log)
        typer.echo("done (motors disabled).")


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
    gravity: Annotated[
        bool,
        typer.Option(
            "--gravity/--no-gravity",
            help="Add a gravity-comp torque feedforward on top of the position hold (ramp its scale live with '-g').",
        ),
    ] = False,
    tau_scale: Annotated[
        float,
        typer.Option(
            "--tau-scale",
            min=0.0,
            max=_MAX_TAU_SCALE,
            help="Initial gravity-comp feedforward scale (0 = off; ramp up live with '-g'). Needs --gravity.",
        ),
    ] = _DEFAULT_TAU_SCALE,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Interactive joint-POSITION REPL. Brings the arm up once, then a background thread streams it
    continuously -- the arm always holds (the stream keeps the DM command-timeout from firing) and moves
    smoothly whenever you retarget a joint, exactly how aegis streams the production driver. Type
    '-jN <rad>' to retarget joints (unset joints keep their target), 'rest' to home, 'stop' to freeze,
    'h' for help, 'q' to park at REST and exit. Holding the bus open the whole session is why this works
    where one-shot commands drop the arm. Pass --gravity to add the g(q) feedforward (stiff move gains stay,
    so it just reduces sag); for the soft backdrivable float test use the 'float' command instead.
    """
    _run_arm_repl(channel, speed, clamp, velocity_mode=False, gravity=gravity, tau_scale=tau_scale)


@app.command("send_jv")
def cmd_send_jv(
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
    Interactive joint-VELOCITY REPL -- the same continuously-streamed hold as send_jp, but each line moves
    joints at a velocity for a required duration, then stops: '-j6 0.3 -d 1' spins joint 6 at 0.3 rad/s for
    1 s then freezes. A duration is mandatory so a joint can never run open-ended into a limit or the
    cameras. 'stop' freezes now, 'rest' ramps home, 'q' parks and exits. Velocity is a ramping MIT position
    command under the same torque clamp, so it stays bounded and holds the other joints (unlike firmware
    VEL). The home / rest ramp uses the default move speed.
    """
    _run_arm_repl(channel, _DEFAULT_SEND_JP_SPEED_RAD_S, clamp, velocity_mode=True)


@app.command("float")
def cmd_float(
    tau_scale: Annotated[
        float,
        typer.Option(
            "-g",
            "--tau-scale",
            min=0.0,
            max=_MAX_TAU_SCALE,
            help="Initial gravity-comp feedforward scale (0 = limp; tune live with '-g <scale>').",
        ),
    ] = _DEFAULT_TAU_SCALE,
    channel: Annotated[str, _CHANNEL_OPTION] = REBOT_B601_DM_DEFAULT_CHANNEL,
) -> None:
    """
    Pure gravity-compensation float: the arm actively carries its own weight (a scaled g(q) torque
    feedforward) with NO position hold (kp = 0), so you can move it around by hand as if it were floating in
    space. This is the gravity-comp test bench. Start at scale 0 (the arm is limp, only lightly damped --
    SUPPORT IT) and raise the scale with '-g <scale>' until the arm hangs weightless wherever you leave it:
    below the right value it sinks under its own weight, above it drifts upward. That best-float scale (a bit
    under 1, the remainder lost to geartrain friction) is what gets baked into the driver. 'q' disables and
    exits.
    """
    _run_float_repl(channel, tau_scale)


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
