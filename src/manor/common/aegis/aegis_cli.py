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
  single block, with no arg for every block applicable to the
  current mode (other blocks are omitted: you can't run them here
  anyway).
* ``aegis repl`` drops into an interactive prompt_toolkit shell that
  exposes the same commands with history + autocomplete. Children
  spawned via ``run`` are SIGTERMed when the REPL exits so nothing
  outlives the supervisor.

Cross-process state lives in PID files under ``/tmp/aegis_*.pid``,
so ``status`` from a fresh shell still works after the REPL exits.

Configuration is the project-bundled YAML
(``configs/aegis/lite6_ac.yaml``). Only ``aegis repl`` accepts
``--config`` / ``-c`` (bare filenames resolve under
``configs/aegis/``; absolute paths are honoured as-is) and
``--mode`` / ``-m``; those flags pin the config + mode for the entire
REPL session. The non-REPL commands (``aegis run``, ``aegis kill``,
``aegis status``) deliberately take no overrides -- the REPL is the
intended interaction surface, the standalone commands exist for
debugging, and consolidating overrides on the REPL keeps the rest
honest (one source of truth per session, no flag drift between
``run`` and ``status``). To run against a different config or in a
different mode outside the REPL, edit the YAML.
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
from prompt_toolkit import PromptSession
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
from prompt_toolkit.completion import WordCompleter
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.history import FileHistory

from manor.common.aegis.aegis import AegisConfig, compose_aegis_yaml_dict
from manor.common.aegis.mode import AegisMode
from manor.common.exceptions import AegisConfigError

# Default YAML path -- resolved relative to the manor repo root, so
# the CLI Just Works whether you launch from the repo root or from a
# subdirectory. Located four parents up from this file:
# src/manor/common/aegis/aegis_cli.py -> .../manor.
_REPO_ROOT = Path(__file__).resolve().parents[4]
_CONFIG_DIR = _REPO_ROOT / "configs" / "aegis"
_DEFAULT_CONFIG_PATH = _CONFIG_DIR / "lite6_ac.yaml"

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

# After SIGTERM times out we escalate to SIGKILL. SIGKILL can't be caught, so the kernel reaps the
# process almost immediately -- 1 s is more than enough margin while keeping the wait responsive.
_SIGKILL_WAIT_TIMEOUT_S = 1.0

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


def _find_orphan_pids(block: AegisBlock) -> list[int]:
    """
    Scan /proc for processes invoked as ``python -m <runner_module>``
    that aren't tracked by ``block``'s PID file. Catches the case where
    a child didn't exit on SIGTERM and the PID file got cleared anyway
    (or where a previous session's process was never killed).

    The match is structural, not a substring grep: argv must look like
    ``[<python>, "-m", "<module>", ...]`` exactly. An editor with the
    runner's *file* open carries the path on disk in its argv (slashes,
    .py extension, positional), not ``-m <module>``, so it won't match
    here. Linux-specific; the project already targets linux so /proc is
    fine.
    """
    module = _BLOCK_RUN_MODULE[block]
    expected_pid = _read_pid(block)
    orphans: list[int] = []
    proc_root = Path("/proc")
    if not proc_root.exists():
        return orphans
    for proc_dir in proc_root.iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        if pid == expected_pid:
            continue
        try:
            cmdline = (proc_dir / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, OSError):
            continue
        # /proc/<pid>/cmdline is null-separated; the trailing null leaves an empty final field.
        argv = [s.decode("utf-8", errors="replace") for s in cmdline.split(b"\x00") if s]
        if len(argv) >= 3 and Path(argv[0]).name.startswith("python") and argv[1] == "-m" and argv[2] == module:
            orphans.append(pid)
    return orphans


def _wait_for_exit(pid: int, timeout_s: float = _KILL_WAIT_TIMEOUT_S) -> bool:
    """
    Block until ``pid`` is gone (or ``timeout_s`` elapses), so any
    last-gasp output the child writes lands before our caller redraws.
    Returns True if the process actually exited within the window.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and _process_alive(pid):
        time.sleep(_KILL_WAIT_POLL_INTERVAL_S)
    return not _process_alive(pid)


def _terminate(pid: int, label: str) -> None:
    """
    Reliably end ``pid``: SIGTERM, wait for graceful exit, escalate to
    SIGKILL if the process is still alive after the timeout. ``label``
    is the user-facing name (e.g. block name + pid) used in warnings.
    Returns when the process is actually gone or after SIGKILL gives
    up too -- but SIGKILL can't be caught, so non-exit after SIGKILL
    means the process is in uninterruptible sleep, which we surface as
    an error rather than silently moving on.
    """
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    if _wait_for_exit(pid):
        return
    _echo_warn(f"{label} did not exit within {_KILL_WAIT_TIMEOUT_S:.0f}s of SIGTERM; sending SIGKILL")
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    if not _wait_for_exit(pid, timeout_s=_SIGKILL_WAIT_TIMEOUT_S):
        _echo_error(
            f"{label} still alive after SIGKILL -- likely uninterruptible kernel sleep "
            f"(stuck in a syscall). Investigate before relaunching."
        )


def _echo_success(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.GREEN)


def _echo_warn(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.YELLOW)


def _echo_error(msg: str) -> None:
    typer.secho(msg, fg=typer.colors.RED, err=True)


def _format_block_state(name: str, pid: Optional[int], orphan_pids: list[int]) -> str:
    if pid is None:
        state = typer.style("stopped", fg=typer.colors.RED)
    else:
        state = typer.style(f"running (pid {pid})", fg=typer.colors.GREEN)
    if not orphan_pids:
        return f"{name}: {state}"
    orphan_list = ", ".join(str(p) for p in orphan_pids)
    return f"{name}: {state} {typer.style(f'[orphan pids: {orphan_list}]', fg=typer.colors.YELLOW)}"


def _reap_orphans(block: AegisBlock) -> int:
    """
    Terminate every orphan PID found for ``block``. Orphans are matched
    structurally (argv == ``[python, -m, <module>, ...]``), so editors
    with the runner file open are not at risk. Returns the number of
    orphans terminated.
    """
    orphans = _find_orphan_pids(block)
    for pid in orphans:
        _echo_warn(f"reaping orphan {block.value} (pid {pid})")
        _terminate(pid, f"orphan {block.value} (pid {pid})")
    return len(orphans)


def _kill_all_running_blocks() -> list[AegisBlock]:
    """
    Stop every block whose PID file points at a live process and clear
    its PID file once the process is actually gone, then sweep up any
    orphan PIDs (runner-module processes not tracked by the PID file).
    Used both as the REPL shutdown hook (so children don't outlive the
    supervisor) and as the no-arg ``kill`` implementation. SIGTERM is
    escalated to SIGKILL inside ``_terminate`` if a child doesn't honour
    the soft signal in time.
    """
    killed: list[AegisBlock] = []
    for block in AegisBlock:
        pid = _read_pid(block)
        if pid is not None:
            _echo_warn(f"signalled {block.value} (pid {pid})")
            _terminate(pid, f"{block.value} (pid {pid})")
            _clear_pid(block)
            _echo_warn(f"stopped {block.value} (pid {pid})")
            killed.append(block)
        if _reap_orphans(block) > 0 and block not in killed:
            killed.append(block)
    return killed


def _resolve_config_path(config_path: Path) -> Path:
    """
    Resolve a ``--config`` argument: bare filenames (and any
    non-absolute path) are looked up under ``configs/aegis/``;
    absolute paths are returned untouched as an escape hatch for
    out-of-tree configs. Aegis configs by convention end with
    ``_ac.yaml`` and live in that directory, so the typical
    invocation is ``--config foo_ac.yaml``.
    """
    if config_path.is_absolute():
        return config_path
    return _CONFIG_DIR / config_path


def _load_config(config_path: Path, mode_override: Optional[AegisMode]) -> tuple[Path, AegisConfig, dict]:
    """
    Read + validate the YAML. Returns the resolved absolute path, the
    parsed ``AegisConfig`` (used to gate which blocks can run), and
    the inlined raw dict (re-serialised to JSON when we spawn each
    child). The base YAML's ``policy_type`` / ``controller_type``
    are resolved against ``policies/`` / ``controllers/`` siblings
    via ``compose_aegis_yaml_dict``; the inlined dict is what the
    runners receive. When ``mode_override`` is given, the YAML's
    ``mode`` field is replaced before validation, so a sim config
    can be coerced to hardware (or vice versa) without editing the
    file on disk.
    """
    config_path = _resolve_config_path(config_path)
    if not config_path.exists():
        raise typer.BadParameter(f"config file not found: {config_path}")
    try:
        raw = compose_aegis_yaml_dict(config_path)
    except AegisConfigError as e:
        raise typer.BadParameter(f"config composition failed: {e}") from e
    if mode_override is not None:
        raw = {**raw, "mode": mode_override.value}
    try:
        config = AegisConfig.from_yaml_dict(raw)
    except AegisConfigError as e:
        raise typer.BadParameter(f"config validation failed: {e}") from e
    return config_path, config, raw


@attr.frozen
class _CliState:
    """
    Per-invocation state shared with subcommands that need a parsed
    config (``run`` and ``repl``). The REPL pins this at launch and
    can refresh it via ``reload``; ``mode_override`` is stashed so a
    reload reproduces the launch-time invocation faithfully (a session
    started with ``--mode hardware`` keeps that override across YAML
    re-reads, instead of silently reverting to the YAML's ``mode``).
    """

    config_path: Path
    config: AegisConfig
    raw_config: dict
    mode_override: Optional[AegisMode] = None


app = typer.Typer(
    name="aegis",
    add_completion=False,
    help="Supervisor / REPL for the aegis robotics stack.",
    # Print help instead of erroring when invoked without a subcommand.
    no_args_is_help=True,
    # Accept ``-h`` as a shorthand for ``--help`` everywhere; click
    # propagates this context_settings dict down to every subcommand
    # so the alias works on ``aegis -h``, ``aegis run -h``, etc.
    context_settings={"help_option_names": ["-h", "--help"]},
)


def _build_state(config_path: Path, mode_override: Optional[AegisMode]) -> _CliState:
    resolved, parsed, raw = _load_config(config_path, mode_override)
    return _CliState(config_path=resolved, config=parsed, raw_config=raw, mode_override=mode_override)


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
    # Same TTY race as ``_kill_impl``'s wait, in the opposite
    # direction: let the child print its startup output before the
    # REPL redraws its prompt.
    time.sleep(_RUN_SETTLE_S)


# --- impl functions ---------------------------------------------------------
#
# The three core operations are factored out as ``_*_impl`` so the
# REPL can invoke them directly with its pinned ``_CliState`` -- no
# flag injection, no re-dispatch through click. The thin click
# wrappers below build state from the bundled default config (no
# overrides; outside the REPL the YAML is the single source of
# truth) and forward to the impls.


def _run_impl(state: _CliState, block: Optional[AegisBlock]) -> None:
    """
    Spawn ``block`` (or every applicable block, if ``block is None``)
    using ``state``'s config + mode. Refuses to start a block that
    isn't part of the configured mode, or one that's already running.
    """
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


def _kill_impl(block: Optional[AegisBlock]) -> None:
    """
    Stop ``block``'s subprocess (or every running block, if ``block is
    None``) and sweep up any orphan PIDs left behind by previous runs
    that didn't clean up. ``kill`` operates purely on PID files +
    /proc cmdline scans -- no state needed.
    """
    if block is None:
        killed = _kill_all_running_blocks()
        if not killed:
            _echo_warn("nothing running")
        return
    pid = _read_pid(block)
    if pid is not None:
        _echo_warn(f"signalled {block.value} (pid {pid})")
        # Block until the child is actually gone (escalating to SIGKILL on timeout) so its
        # SIGTERM-handler print ("received signal 15; stopping") lands before the REPL redraws the
        # next prompt and so we never clear the PID file while the process is still around.
        _terminate(pid, f"{block.value} (pid {pid})")
        _clear_pid(block)
        _echo_warn(f"stopped {block.value} (pid {pid})")
    reaped = _reap_orphans(block)
    if pid is None and reaped == 0:
        _echo_error(f"{block.value} is not running")
        raise typer.Exit(code=1)


def _status_impl(state: _CliState, block: Optional[AegisBlock]) -> None:
    """
    Report block state. With ``block``, prints just that block's PID
    state. With no block, prints a mode banner followed by every
    block applicable to ``state.config.mode``; non-applicable blocks
    are omitted entirely. Each line also surfaces any orphan PIDs --
    runner-module processes not tracked by the PID file -- so a child
    that survived its kill (or was never killed) doesn't sit invisible.
    """
    if block is not None:
        typer.echo(_format_block_state(block.value, _read_pid(block), _find_orphan_pids(block)))
        return
    allowed = _MODE_BLOCKS[state.config.mode]
    typer.secho(f"mode: {state.config.mode.value}", fg=typer.colors.MAGENTA, bold=True)
    for b in sorted(allowed, key=lambda x: x.value):
        typer.echo(f"  {_format_block_state(b.value, _read_pid(b), _find_orphan_pids(b))}")


def _reload_impl(state: _CliState) -> _CliState:
    """
    Re-read the YAML configs from disk and rebuild the REPL's pinned
    state. The launch-time ``mode_override`` (if any) is preserved
    so a session started with ``--mode hardware`` keeps that override
    after the reload.

    Already-running blocks keep the stale config they were spawned
    with -- the reload only affects blocks spawned *after* it. The
    typical workflow is: kill the relevant block, edit the YAML or
    a sub-YAML, ``reload``, then ``run`` it again. We warn (not
    error) when blocks are still up so the user catches the case
    where they expected the change to take effect immediately.

    On parse / validation failure the previous state is kept and
    the error is echoed -- a half-broken edit shouldn't drop the
    user out of the REPL.
    """
    try:
        new_state = _build_state(state.config_path, state.mode_override)
    except typer.BadParameter as e:
        _echo_error(f"reload failed: {e}")
        return state
    except AegisConfigError as e:
        _echo_error(f"reload failed: {e}")
        return state
    running = [b for b in AegisBlock if _read_pid(b) is not None]
    banner = f"reloaded -- mode={new_state.config.mode.value}, config={new_state.config_path}"
    if running:
        _echo_warn(banner)
        _echo_warn(
            f"warning: {sorted(b.value for b in running)} still running with the previous "
            f"config; restart them to pick up the reload"
        )
    else:
        _echo_success(banner)
    return new_state


# --- click wrappers ---------------------------------------------------------
#
# Outside the REPL, these are how ``aegis run`` / ``aegis kill`` /
# ``aegis status`` enter from the command line. They take no config
# / mode flags -- the bundled YAML is the single source of truth.
# To run a different config or flip mode, edit the YAML (or use the
# REPL, which does take ``--config`` / ``--mode``).


@app.command("run")
def run_block(
    block: Annotated[
        Optional[AegisBlock],
        typer.Argument(help="Which aegis block to start. Omit to start every block applicable to this mode."),
    ] = None,
) -> None:
    """
    Spawn ``block`` as a subprocess (or every applicable block with
    no arg), against the bundled default config. Edit the YAML to
    change mode; this command takes no overrides.
    """
    state = _build_state(_DEFAULT_CONFIG_PATH, mode_override=None)
    _run_impl(state, block)


@app.command("kill")
def kill_block(
    block: Annotated[
        Optional[AegisBlock],
        typer.Argument(help="Which aegis block to stop. Omit to stop every running block."),
    ] = None,
) -> None:
    """
    Send SIGTERM to ``block``'s subprocess (or every running block
    with no arg).
    """
    _kill_impl(block)


@app.command("status")
def status(
    block: Annotated[
        Optional[AegisBlock],
        typer.Argument(help="Which block to query. Omit to list every block applicable to the current mode."),
    ] = None,
) -> None:
    """
    Report block state. With no arg, prints a mode banner and lists
    every block applicable to the bundled default config's mode; the
    other blocks are omitted (you can't run them in this mode anyway).
    """
    state = _build_state(_DEFAULT_CONFIG_PATH, mode_override=None)
    _status_impl(state, block)


# Where the REPL stores its history (~/.aegis_history). Persistent
# across invocations -- arrow-up recall just works.
_HISTORY_PATH = Path.home() / ".aegis_history"


@app.command("repl")
def repl(
    config: Annotated[
        Path,
        typer.Option(
            "--config",
            "-c",
            help=(
                "Aegis YAML to load for this REPL session. Bare "
                "filenames resolve under configs/aegis/ (e.g. -c "
                "foo_ac.yaml); absolute paths are honoured as-is. "
                "Defaults to lite6_ac.yaml."
            ),
        ),
    ] = _DEFAULT_CONFIG_PATH,
    mode: Annotated[
        Optional[AegisMode],
        typer.Option(
            "--mode",
            "-m",
            help="Override the YAML's mode field (sim or hardware) for this REPL session.",
        ),
    ] = None,
) -> None:
    """
    Start an interactive shell. The ``--config`` and ``--mode`` flags
    are accepted only here -- they pin the supervisor's view of the
    aegis stack for the entire REPL session, and the ``run`` /
    ``status`` / ``kill`` lines typed inside inherit that view.
    External ``aegis run`` / ``aegis status`` invocations (outside
    the REPL) don't take these flags; edit the YAML to change them.
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
            state = _dispatch_repl_line(line, state)
    finally:
        _kill_all_running_blocks()


def _build_completer() -> WordCompleter:
    """
    Tab-completion vocabulary: top-level commands plus block names.
    Good enough that ``run g<TAB>`` finishes to ``run gylos``.
    """
    words = ["run", "kill", "status", "reload", "help", "exit", "quit", "q"]
    words.extend(block.value for block in AegisBlock)
    return WordCompleter(words, ignore_case=True)


_HELP_FLAGS = frozenset({"-h", "--help"})


def _print_repl_help() -> None:
    """
    Top-level REPL help. Custom (rather than click-generated) because
    this list also covers REPL-only words like ``help`` / ``exit`` /
    ``quit`` / ``q`` (and ``reload``) that aren't click subcommands.
    """
    typer.echo("commands:")
    typer.echo("  run [block]      spawn a block as a subprocess (no arg = all applicable)")
    typer.echo("  kill [block]     SIGTERM a running block (no arg = all running)")
    typer.echo("  status [block]   report block state (no arg = all applicable)")
    typer.echo("  reload           re-read the YAML configs from disk (already-running blocks keep old config)")
    typer.echo("  help | -h        show this message")
    typer.echo("  exit | quit | q  leave the REPL (running blocks are stopped)")
    typer.echo("")
    typer.echo("type '<command> -h' for help on a specific command (e.g. 'run -h').")


def _print_reload_help() -> None:
    """
    REPL-only ``reload`` has no click counterpart, so its help has to
    be hand-rolled. Format mirrors click's ``Usage:`` block so the
    visual cue is consistent across commands.
    """
    typer.echo("Usage: aegis reload")
    typer.echo("")
    typer.echo("  Re-read the base YAML and policy / controller sub-YAMLs from disk and rebuild")
    typer.echo("  the REPL's pinned config. The launch-time --mode override (if any) is")
    typer.echo("  preserved. Already-running blocks keep their old config until restarted.")
    typer.echo("")
    typer.echo("  Typical use: kill a block, edit a YAML, reload, run the block again -- without")
    typer.echo("  exiting the REPL.")


def _print_command_help(cmd_name: str) -> None:
    """
    Render the click command's auto-generated help, the same output
    you get from ``aegis <cmd> -h`` outside the REPL. Keeps per-
    command help formatting consistent across both entry points.
    """
    click_cmd = cli.commands.get(cmd_name)
    if click_cmd is None:
        _echo_error(f"no help available for {cmd_name!r}")
        return
    # The parent context gives the Usage line its ``aegis`` prefix
    # (click walks up the parent chain to render the full command
    # path). ``help_option_names`` has to be set on the parent we
    # build by hand: the typer app declares it on its top-level
    # context_settings, but a manually-built parent context doesn't
    # pull from the group's settings, so without this the help
    # banner shows only ``--help`` instead of the matched
    # ``-h, --help`` pair seen on the standalone CLI.
    parent_ctx = click.Context(cli, info_name="aegis", help_option_names=["-h", "--help"])
    ctx = click.Context(click_cmd, info_name=cmd_name, parent=parent_ctx)
    typer.echo(click_cmd.get_help(ctx))


_REPL_BLOCK_COMMANDS = {"run", "kill", "status"}
_REPL_NULLARY_COMMANDS = {"reload"}


def _parse_repl_block_arg(argv: list[str]) -> Optional[AegisBlock]:
    """
    Parse the optional block argument on a REPL line. ``argv[0]`` is
    the command (``run`` / ``kill`` / ``status``); ``argv[1:]`` is
    the block name, if any. Raises ``typer.BadParameter`` (caught by
    the dispatch loop) on bad input so the REPL keeps running.
    """
    if len(argv) > 2:
        raise typer.BadParameter(f"{argv[0]}: too many arguments (expected at most 1, got {len(argv) - 1})")
    if len(argv) == 1:
        return None
    raw = argv[1]
    try:
        return AegisBlock(raw)
    except ValueError:
        valid = ", ".join(b.value for b in AegisBlock)
        raise typer.BadParameter(f"{argv[0]}: unknown block {raw!r}; valid: {valid}") from None


def _dispatch_repl_line(line: str, state: _CliState) -> _CliState:
    """
    Parse a REPL line and call the matching impl function with the
    REPL's pinned ``state``. Returns the (possibly updated) state so
    the REPL loop can re-bind it -- used by ``reload`` to refresh
    state without restarting the REPL. Other commands return the
    same state instance unchanged.

    We don't go through click here -- the REPL doesn't accept
    ``--config`` / ``--mode`` per-line (those are pinned at REPL
    launch), so the click flag-parsing layer would just be in the
    way. ``-h`` / ``--help`` on its own re-prints the top-level
    help; on a command (``run -h``, etc.) it prints that command's
    help. Errors are caught and echoed instead of bubbling up, so a
    bad command doesn't kill the REPL.
    """
    try:
        argv = shlex.split(line)
    except ValueError as e:
        _echo_error(f"parse error: {e}")
        return state
    if not argv:
        return state
    if argv[0] in _HELP_FLAGS:
        _print_repl_help()
        return state
    cmd = argv[0]
    if cmd == "repl":
        _echo_error("already in REPL")
        return state
    if cmd in _REPL_NULLARY_COMMANDS:
        if any(a in _HELP_FLAGS for a in argv[1:]):
            if cmd == "reload":
                _print_reload_help()
            return state
        if len(argv) > 1:
            _echo_error(f"{cmd}: takes no arguments (got {argv[1:]!r})")
            return state
        if cmd == "reload":
            return _reload_impl(state)
        return state
    if cmd not in _REPL_BLOCK_COMMANDS:
        _echo_error(f"unknown command: {cmd!r}; type 'help' for the list")
        return state
    if any(a in _HELP_FLAGS for a in argv[1:]):
        # Mirror click's behaviour: a help flag anywhere in the
        # remaining argv short-circuits to per-command help.
        _print_command_help(cmd)
        return state
    try:
        block = _parse_repl_block_arg(argv)
    except typer.BadParameter as e:
        _echo_error(str(e))
        return state
    try:
        if cmd == "run":
            _run_impl(state, block)
        elif cmd == "status":
            _status_impl(state, block)
        else:
            _kill_impl(block)
    except typer.Exit:
        # Impls raise typer.Exit after echoing their own error
        # message; suppress so the REPL keeps running.
        pass
    return state


# typer apps are click apps under the hood; expose the click
# entry-point as ``cli`` for the pyproject script declaration and
# for the REPL re-dispatch above.
cli = typer.main.get_command(app)


if __name__ == "__main__":
    cli()
