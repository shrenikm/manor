"""
``aegis`` -- the top-level CLI / REPL for the aegis stack.

The aegis stack runs as N independent OS processes (one per logical
block: ``metis``, ``gylos`` in sim; ``metis``, ``kylos``, ``helios``
on hardware). This module is the supervisor for those processes:

* ``aegis run <block>`` spawns a child running
  ``python -m manor.common.aegis.run.run_<block>`` with the parsed +
  validated config piped in over stdin as JSON.
* ``aegis kill <block>`` sends SIGTERM to the child.
* ``aegis status [block]`` reports state -- with ``<block>`` for a
  single block, with no arg for every known block.
* ``aegis repl`` drops into an interactive prompt_toolkit shell that
  exposes the same commands with history + autocomplete. Children
  spawned via ``run`` are SIGTERMed when the REPL exits so nothing
  outlives the supervisor.

Cross-process state lives in PID files under ``/tmp/aegis_*.pid``,
so ``status`` from a fresh shell still works after the REPL exits.

Configuration is the project-bundled YAML
(``configs/aegis/default_ac.yaml``). ``run`` and ``repl`` accept
``--config`` / ``-c`` to load a different aegis config and
``--mode`` / ``-m`` to override the YAML's ``mode`` field at the
command line. ``kill`` and ``status`` operate purely on the PID
files, so they don't take config or mode flags.
"""

from __future__ import annotations

import json
import os
import shlex
import signal
import subprocess
import sys
import time
from enum import StrEnum
from pathlib import Path
from typing import Annotated, Optional

import attr
import click
import typer
import yaml
from prompt_toolkit import PromptSession
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
from prompt_toolkit.completion import WordCompleter
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.history import FileHistory

from manor.common.aegis.aegis import AegisConfig
from manor.common.aegis.mode import AegisMode
from manor.common.exceptions import AegisConfigError

# Default YAML path -- resolved relative to the manor repo root, so
# the CLI Just Works whether you launch from the repo root or from a
# subdirectory. Located four parents up from this file:
# src/manor/common/aegis/aegis_cli.py -> .../manor.
_REPO_ROOT = Path(__file__).resolve().parents[4]
_DEFAULT_CONFIG_PATH = _REPO_ROOT / "configs" / "aegis" / "default_ac.yaml"

# PID files live in /tmp, namespaced per-block so concurrent aegis
# stacks (e.g. CI + local dev) don't collide. Keep a stable prefix
# so ``aegis list`` can enumerate them with a glob.
_PID_FILE_DIR = Path("/tmp")
_PID_FILE_PREFIX = "aegis_"
_PID_FILE_SUFFIX = ".pid"


# After SIGTERM, wait this long for the child to actually exit before
# returning. Without this, the child's own ``stopping`` print races
# against the REPL's next prompt redraw and lands on top of it.
# Generous because gylos has to tear down Drake Meshcat (a C++ server
# thread) before the process truly exits; if the next REPL invocation
# is racing it, the meshcat listening socket may not yet be released.
_KILL_WAIT_TIMEOUT_S = 5.0
_KILL_WAIT_POLL_INTERVAL_S = 0.02

# After spawning a block, sleep this long before returning so the
# child's startup output (Drake's "Meshcat listening at ..." banner,
# in particular) lands on the TTY before the REPL redraws its prompt.
# Without this, the child's stdout writes overlay the new prompt and
# the cursor strands until the user hits Enter.
_RUN_SETTLE_S = 1.5


class AegisBlock(StrEnum):
    """
    Named blocks the supervisor can spawn. ``metis`` and ``gylos``
    are the sim-mode pair; ``metis``, ``kylos``, and ``helios`` are
    the hardware-mode triad.
    """

    METIS = "metis"
    GYLOS = "gylos"
    KYLOS = "kylos"
    HELIOS = "helios"


# Per-block run-module path (the ``-m`` argument we hand to python).
_BLOCK_RUN_MODULE: dict[AegisBlock, str] = {
    AegisBlock.METIS: "manor.common.aegis.run.run_metis",
    AegisBlock.GYLOS: "manor.common.aegis.run.run_gylos",
    AegisBlock.KYLOS: "manor.common.aegis.run.run_kylos",
    AegisBlock.HELIOS: "manor.common.aegis.run.run_helios",
}


# Which blocks each AegisMode actually runs. The REPL refuses to
# spawn blocks that don't apply to the configured mode, with a
# pointed error message explaining the alternative.
_MODE_BLOCKS: dict[AegisMode, set[AegisBlock]] = {
    AegisMode.SIM: {AegisBlock.METIS, AegisBlock.GYLOS},
    AegisMode.HARDWARE: {AegisBlock.METIS, AegisBlock.KYLOS, AegisBlock.HELIOS},
}


def _pid_file_path(block: AegisBlock) -> Path:
    return _PID_FILE_DIR / f"{_PID_FILE_PREFIX}{block.value}{_PID_FILE_SUFFIX}"


def _read_pid(block: AegisBlock) -> Optional[int]:
    """
    Return the PID stored for ``block`` if the PID file exists and
    refers to a live process; otherwise clear any stale file and
    return None.
    """
    path = _pid_file_path(block)
    if not path.exists():
        return None
    try:
        pid = int(path.read_text().strip())
    except (OSError, ValueError):
        return None
    if not _process_alive(pid):
        # Stale file (process exited without cleaning up); remove so
        # ``run`` doesn't refuse to start due to a phantom record.
        try:
            path.unlink()
        except OSError:
            pass
        return None
    return pid


def _write_pid(block: AegisBlock, pid: int) -> None:
    _pid_file_path(block).write_text(str(pid))


def _clear_pid(block: AegisBlock) -> None:
    path = _pid_file_path(block)
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def _process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # Process exists but isn't ours to signal; still counts as alive.
        return True
    return True


def _wait_for_exit(pid: int, timeout_s: float = _KILL_WAIT_TIMEOUT_S) -> None:
    """
    Block until ``pid`` is gone (or ``timeout_s`` elapses), so any
    last-gasp output the child writes lands before our caller redraws.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and _process_alive(pid):
        time.sleep(_KILL_WAIT_POLL_INTERVAL_S)


def _echo_success(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.GREEN)


def _echo_warn(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.YELLOW)


def _echo_error(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.RED, err=True)


def _format_block_state(name: str, pid: Optional[int]) -> str:
    if pid is None:
        return f"{name}: {typer.style('stopped', fg=typer.colors.RED)}"
    return f"{name}: {typer.style(f'running (pid {pid})', fg=typer.colors.GREEN)}"


def _kill_all_running_blocks() -> list[AegisBlock]:
    """
    SIGTERM every block whose PID file points at a live process, wait
    for each to exit, and clear its PID file. Returns the list of
    blocks that were actually signalled. Used both as the REPL
    shutdown hook (so children don't outlive the supervisor) and as
    the no-arg ``kill`` implementation. Output mirrors per-block
    ``kill_block`` so the log reads like a series of normal kills.
    """
    pending: list[tuple[AegisBlock, int]] = []
    for block in AegisBlock:
        pid = _read_pid(block)
        if pid is None:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            _clear_pid(block)
            continue
        _echo_warn(f"signalled {block.value} (pid {pid})")
        pending.append((block, pid))
    for block, pid in pending:
        _wait_for_exit(pid)
        _clear_pid(block)
        _echo_warn(f"stopped {block.value} (pid {pid})")
    return [b for b, _ in pending]


def _load_config(config_path: Path, mode_override: Optional[AegisMode]) -> tuple[AegisConfig, dict]:
    """
    Read + validate the YAML. Returns the parsed ``AegisConfig`` (used
    to gate which blocks can run) plus the raw dict (re-serialised to
    JSON when we spawn each child). When ``mode_override`` is given,
    the YAML's ``mode`` field is replaced before validation, so a sim
    config can be coerced to hardware (or vice versa) without editing
    the file on disk.
    """
    if not config_path.exists():
        raise typer.BadParameter(f"config file not found: {config_path}")
    with open(config_path, "r") as fp:
        raw = yaml.safe_load(fp) or {}
    if not isinstance(raw, dict):
        raise typer.BadParameter(f"config {config_path} must be a top-level mapping")
    if mode_override is not None:
        raw = {**raw, "mode": mode_override.value}
    try:
        config = AegisConfig.from_yaml_dict(raw)
    except AegisConfigError as e:
        raise typer.BadParameter(f"config validation failed: {e}") from e
    return config, raw


@attr.frozen
class _CliState:
    """
    Per-invocation state shared with subcommands that need a parsed
    config (``run`` and ``repl``). Reconstructed from disk inside the
    REPL whenever it re-dispatches a ``run`` line.
    """

    config_path: Path
    config: AegisConfig
    raw_config: dict


# Shared option specs so ``run`` and ``repl`` declare ``--config`` /
# ``--mode`` identically. ``kill`` and ``status`` operate on PID
# files alone and don't take these flags.
_CONFIG_OPTION = typer.Option(
    "--config",
    "-c",
    help="Path to the aegis YAML. Defaults to configs/aegis/default_ac.yaml.",
)
_MODE_OPTION = typer.Option(
    "--mode",
    "-m",
    help="Override the YAML's mode field (sim or hardware) without editing the file.",
)


app = typer.Typer(
    name="aegis",
    add_completion=False,
    help="Supervisor / REPL for the aegis robotics stack.",
    # Accept ``-h`` as a shorthand for ``--help`` everywhere; click
    # propagates this context_settings dict down to every subcommand
    # so the alias works on ``aegis -h``, ``aegis run -h``, etc.
    context_settings={"help_option_names": ["-h", "--help"]},
)


@app.command("run")
def run_block(
    block: Annotated[
        Optional[AegisBlock],
        typer.Argument(help="Which aegis block to start. Omit to start every block applicable to this mode."),
    ] = None,
    config: Annotated[Path, _CONFIG_OPTION] = _DEFAULT_CONFIG_PATH,
    mode: Annotated[Optional[AegisMode], _MODE_OPTION] = None,
) -> None:
    """
    Spawn ``block`` as a subprocess. With no argument, spawns every
    block applicable to the current mode that isn't already running.
    Each child is given the parsed AegisConfig over stdin as JSON.
    Refuses to start a block that doesn't apply to the configured
    mode, or one that's already running (per the PID file).
    """
    state = _build_state(config, mode)
    allowed = _MODE_BLOCKS[state.config.mode]

    if block is None:
        # No-arg path: start every applicable block that isn't
        # already running. Already-running blocks are warnings, not
        # errors -- in batch mode a duplicate ``run`` shouldn't abort
        # the rest of the start sequence.
        started = 0
        for b in sorted(allowed, key=lambda x: x.value):
            existing = _read_pid(b)
            if existing is not None:
                _echo_warn(f"{b.value} already running (pid {existing})")
                continue
            _spawn_block(state, b)
            started += 1
        if started == 0:
            _echo_warn("nothing to start")
        return

    if block not in allowed:
        _echo_error(
            f"refusing to run {block.value!r} in mode {state.config.mode.value!r}; "
            f"this mode supports {sorted(b.value for b in allowed)}"
        )
        raise typer.Exit(code=1)

    if (existing := _read_pid(block)) is not None:
        _echo_error(f"{block.value} already running (pid {existing})")
        raise typer.Exit(code=1)

    _spawn_block(state, block)


def _build_state(config_path: Path, mode_override: Optional[AegisMode]) -> _CliState:
    parsed, raw = _load_config(config_path, mode_override)
    return _CliState(config_path=config_path, config=parsed, raw_config=raw)


def _spawn_block(state: _CliState, block: AegisBlock) -> None:
    """
    Spawn one block as a subprocess, write its PID file, and pause
    briefly so the child's startup output (Drake's Meshcat URL
    banner, etc.) lands on the TTY before control returns to the
    caller. Caller is responsible for verifying the block is
    applicable to the current mode and not already running.
    """
    payload = json.dumps(state.raw_config).encode("utf-8")
    proc = subprocess.Popen(
        [sys.executable, "-m", _BLOCK_RUN_MODULE[block]],
        stdin=subprocess.PIPE,
        # Detach from the supervisor's stdout/stderr so the REPL
        # doesn't get spammed with the child's logs; the child can
        # still write to its own descriptors.
        stdout=None,
        stderr=None,
    )
    assert proc.stdin is not None
    proc.stdin.write(payload)
    proc.stdin.close()
    _write_pid(block, proc.pid)
    _echo_success(f"started {block.value} (pid {proc.pid})")
    # Same TTY race as ``kill_block``'s wait, in the opposite
    # direction: let the child print its startup output before the
    # REPL redraws its prompt.
    time.sleep(_RUN_SETTLE_S)


@app.command("kill")
def kill_block(
    block: Annotated[
        Optional[AegisBlock],
        typer.Argument(help="Which aegis block to stop. Omit to stop every running block."),
    ] = None,
) -> None:
    """
    Send SIGTERM to ``block``'s subprocess. With no argument, signals
    every block currently running and waits for each to exit. The
    child has its own SIGTERM handler that exits cleanly; this
    command just signals + waits.
    """
    if block is None:
        killed = _kill_all_running_blocks()
        if not killed:
            _echo_warn("nothing running")
        return
    pid = _read_pid(block)
    if pid is None:
        _echo_error(f"{block.value} is not running")
        raise typer.Exit(code=1)
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        # Race: process exited between the read_pid check and the kill.
        _clear_pid(block)
        _echo_warn(f"{block.value} already exited")
        return
    _clear_pid(block)
    # Block until the child is gone so its own SIGTERM-handler print
    # ("received signal 15; stopping") lands before the REPL redraws
    # the next prompt. Without this the child's last line writes onto
    # the line where ``aegis>`` already is and strands the cursor.
    _wait_for_exit(pid)
    _echo_warn(f"signalled {block.value} (pid {pid})")


@app.command("status")
def status(
    block: Annotated[
        Optional[AegisBlock],
        typer.Argument(help="Which block to query. Omit to report every known block."),
    ] = None,
) -> None:
    """
    Report block state. With no argument, lists every known block
    along with its state (running + PID, or stopped). With ``<block>``,
    reports just that block. ``status`` reads PID files directly and
    therefore doesn't need a config or mode -- it shows whatever is
    running, regardless of how it was launched.
    """
    if block is not None:
        typer.echo(_format_block_state(block.value, _read_pid(block)))
        return
    for b in sorted(AegisBlock, key=lambda x: x.value):
        typer.echo(_format_block_state(b.value, _read_pid(b)))


# Where the REPL stores its history (~/.aegis_history). Persistent
# across invocations -- arrow-up recall just works.
_HISTORY_PATH = Path.home() / ".aegis_history"


@app.command("repl")
def repl(
    config: Annotated[Path, _CONFIG_OPTION] = _DEFAULT_CONFIG_PATH,
    mode: Annotated[Optional[AegisMode], _MODE_OPTION] = None,
) -> None:
    """
    Start an interactive shell. All subcommands are available in it.
    The ``--config`` and ``--mode`` flags pinned at REPL launch are
    re-injected into every ``run`` line typed inside, so children
    spawn against the same config the REPL was started with.
    """
    state = _build_state(config, mode)
    typer.secho(
        f"aegis repl -- mode={state.config.mode.value}, config={state.config_path}",
        fg=typer.colors.CYAN,
        bold=True,
    )
    typer.echo("type 'help' for commands, 'exit' or Ctrl-D to leave (running blocks are stopped).")

    completer = _build_completer()
    session: PromptSession[str] = PromptSession(
        history=FileHistory(str(_HISTORY_PATH)),
        auto_suggest=AutoSuggestFromHistory(),
        completer=completer,
        complete_while_typing=True,
    )
    prompt_text = HTML("<ansicyan><b>aegis &gt;&gt;</b></ansicyan> ")

    # The try/finally guarantees ``_kill_all_running_blocks`` runs on
    # every exit path (typed 'exit', Ctrl-D, unexpected exception) so
    # subprocess children don't outlive the supervisor.
    try:
        while True:
            try:
                line = session.prompt(prompt_text).strip()
            except EOFError:
                typer.echo()
                return
            except KeyboardInterrupt:
                # Mirror bash: Ctrl-C clears the line, doesn't exit.
                continue
            if not line:
                continue
            if line in {"exit", "quit", "q"}:
                return
            if line == "help":
                _print_repl_help()
                continue
            _dispatch_repl_line(line, state.config_path, state.config.mode)
    finally:
        _kill_all_running_blocks()


def _build_completer() -> WordCompleter:
    """
    Tab-completion vocabulary: top-level commands plus block names.
    Good enough that ``run g<TAB>`` finishes to ``run gylos``.
    """
    words = ["run", "kill", "status", "help", "exit", "quit", "q"]
    words.extend(block.value for block in AegisBlock)
    return WordCompleter(words, ignore_case=True)


def _print_repl_help() -> None:
    typer.echo("commands:")
    typer.echo("  run [block]      spawn a block as a subprocess (no arg = all applicable)")
    typer.echo("  kill [block]     SIGTERM a running block (no arg = all running)")
    typer.echo("  status [block]   report block state (no arg = all blocks)")
    typer.echo("  help             show this message")
    typer.echo("  exit | quit | q  leave the REPL (running blocks are stopped)")


def _dispatch_repl_line(line: str, config_path: Path, mode: AegisMode) -> None:
    """
    Parse a REPL line as argv and invoke the corresponding click
    subcommand. For ``run`` we re-inject ``--config`` and ``--mode``
    so each child spawns against the same config the outer REPL was
    launched with. ``kill`` and ``status`` consult PID files only and
    take no such flags.

    Errors are caught and echoed instead of bubbling up, so a bad
    command doesn't kill the REPL.
    """
    try:
        argv = shlex.split(line)
    except ValueError as e:
        typer.echo(f"parse error: {e}", err=True)
        return
    if not argv:
        return
    if argv[0] == "repl":
        _echo_error("already in REPL")
        return
    if argv[0] == "run":
        # Pin the REPL's config + mode onto every ``run`` line. click
        # takes the last value for an option, so appending here means
        # any user-supplied ``--config`` / ``--mode`` typed inside the
        # REPL is intentionally overridden -- the REPL is the source
        # of truth for this session; relaunch the REPL to switch.
        full_argv = [*argv, "--config", str(config_path), "--mode", mode.value]
    else:
        full_argv = argv
    try:
        # standalone_mode=False stops click from calling sys.exit on
        # its own; we handle the SystemExit it raises on --help / etc.
        cli.main(args=full_argv, standalone_mode=False, prog_name="aegis")
    except click.UsageError as e:
        _echo_error(f"usage: {e.format_message()}")
    except click.exceptions.Exit:
        # ``raise typer.Exit`` from a subcommand -- already echoed
        # whatever it needed to.
        pass
    except SystemExit:
        pass


# typer apps are click apps under the hood; expose the click
# entry-point as ``cli`` for the pyproject script declaration and
# for the REPL re-dispatch above.
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
