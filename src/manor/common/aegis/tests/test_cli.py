"""
Tests for the ``aegis`` CLI / REPL.

The standalone CLI commands (``aegis run`` / ``aegis kill`` /
``aegis status``) are exercised via click's ``CliRunner``. The REPL
itself isn't driven from pytest -- instead we test the impl
functions (``_run_impl`` / ``_status_impl`` / ``_kill_impl``) and
the parser / state builder it composes, so the REPL's behaviour is
covered without spinning up prompt_toolkit.

PID-file state is sandboxed by pointing ``_PID_FILE_DIR`` at a
``tmp_path`` for each test.
"""

from __future__ import annotations

import os
from pathlib import Path
from unittest import mock

import pytest
import typer
from click.testing import CliRunner

from manor.common.aegis import aegis_cli as cli_module
from manor.common.aegis.aegis_cli import AegisBlock, cli
from manor.common.aegis.mode import AegisMode
from manor.common.testing_utils import run_manor_tests


def _bundled_config_path() -> Path:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    return Path(repo_root) / "configs" / "aegis" / "default_ac.yaml"


@pytest.fixture
def sandboxed_pid_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """
    Redirect PID files into a per-test tmp directory so concurrent
    test runs (and stray processes from a previous local run) don't
    cross-contaminate the assertions.
    """
    monkeypatch.setattr(cli_module, "_PID_FILE_DIR", tmp_path)
    return tmp_path


def _run_cli(args: list[str]) -> "CliRunner.Result":
    runner = CliRunner()
    return runner.invoke(cli, args)


class TestStandaloneCli:
    """
    The non-REPL commands always read the bundled default config; no
    overrides are accepted. The bundled default is sim mode.
    """

    def test_status_no_arg_lists_only_applicable_blocks(self, sandboxed_pid_dir: Path) -> None:
        # Sim mode: status prints a banner + the applicable blocks
        # (metis, gylos). Hardware-only blocks are omitted entirely
        # rather than labelled "unavailable".
        result = _run_cli(["status"])
        assert result.exit_code == 0, result.output
        assert "mode: sim" in result.output
        assert "metis: stopped" in result.output
        assert "gylos: stopped" in result.output
        assert "kylos" not in result.output
        assert "helios" not in result.output

    def test_status_unknown_block_rejected(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status", "not_a_block"])
        assert result.exit_code != 0

    def test_status_reports_stopped_for_known_block(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status", "metis"])
        assert result.exit_code == 0
        assert "metis" in result.output
        assert "stopped" in result.output

    def test_kill_without_running_block_errors(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["kill", "metis"])
        assert result.exit_code != 0
        assert "is not running" in result.output

    def test_kill_no_arg_with_nothing_running_is_noop(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["kill"])
        assert result.exit_code == 0
        assert "nothing running" in result.output

    def test_run_refuses_hardware_block_in_sim_mode(self, sandboxed_pid_dir: Path) -> None:
        # Bundled default is sim mode; kylos is hardware-only.
        result = _run_cli(["run", "kylos"])
        assert result.exit_code != 0
        assert "refusing to run" in result.output

    def test_pid_file_path_is_block_specific(self, sandboxed_pid_dir: Path) -> None:
        path = cli_module._pid_file_path(AegisBlock.METIS)
        assert path.parent == sandboxed_pid_dir
        assert path.name == "aegis_metis.pid"

    def test_run_no_arg_spawns_every_applicable_block(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        spawned: list[str] = []

        def fake_popen(args: list[str], **_kwargs: object) -> mock.MagicMock:
            spawned.append(args[2])
            proc = mock.MagicMock()
            proc.pid = 90000 + len(spawned)
            return proc

        monkeypatch.setattr(cli_module.subprocess, "Popen", fake_popen)
        monkeypatch.setattr(cli_module, "_RUN_SETTLE_S", 0.0)

        result = _run_cli(["run"])
        assert result.exit_code == 0, result.output
        assert "manor.common.aegis.run.run_metis" in spawned
        assert "manor.common.aegis.run.run_gylos" in spawned
        assert "started metis" in result.output
        assert "started gylos" in result.output

    def test_run_no_arg_with_all_blocks_running_is_noop(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        monkeypatch.setattr(cli_module, "_process_alive", lambda _pid: True)
        for block in (AegisBlock.METIS, AegisBlock.GYLOS):
            cli_module._write_pid(block, 12345)

        result = _run_cli(["run"])
        assert result.exit_code == 0
        assert "already running" in result.output
        assert "nothing to start" in result.output

    @pytest.mark.parametrize("command", ["run", "kill", "status"])
    @pytest.mark.parametrize("flag", ["--config", "-c", "--mode", "-m"])
    def test_standalone_commands_reject_override_flags(
        self,
        command: str,
        flag: str,
        sandboxed_pid_dir: Path,
    ) -> None:
        # ``--config`` / ``--mode`` are REPL-only. Outside the REPL
        # the YAML is the single source of truth; passing override
        # flags must fail at parse time so users get a clear error
        # instead of silent fallback.
        result = _run_cli([command, flag, "anything"])
        assert result.exit_code != 0


class TestConfigResolution:
    """
    The ``--config`` resolution helper used by ``aegis repl -c ...``.
    Standalone commands don't accept the flag, but the resolver is
    still public-ish and worth covering directly.
    """

    def test_bare_filename_resolves_under_configs_dir(self) -> None:
        resolved = cli_module._resolve_config_path(Path("default_ac.yaml"))
        assert resolved == _bundled_config_path()

    def test_absolute_path_passes_through(self) -> None:
        abs_path = _bundled_config_path()
        assert cli_module._resolve_config_path(abs_path) == abs_path

    def test_missing_file_errors(self) -> None:
        with pytest.raises(typer.BadParameter, match="config file not found"):
            cli_module._load_config(Path("does_not_exist_ac.yaml"), mode_override=None)


class TestReplPinnedState:
    """
    The REPL is wired around ``_build_state`` (called once at REPL
    launch) plus the ``_*_impl`` functions (called per line).
    Driving prompt_toolkit from pytest is hostile, so we cover the
    moving parts directly with the pinned state.
    """

    def test_build_state_applies_mode_override(self) -> None:
        state = cli_module._build_state(_bundled_config_path(), AegisMode.HARDWARE)
        assert state.config.mode is AegisMode.HARDWARE

    def test_run_impl_in_hardware_mode_spawns_hardware_triad(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        spawned: list[str] = []

        def fake_popen(args: list[str], **_kwargs: object) -> mock.MagicMock:
            spawned.append(args[2])
            proc = mock.MagicMock()
            proc.pid = 80000 + len(spawned)
            return proc

        monkeypatch.setattr(cli_module.subprocess, "Popen", fake_popen)
        monkeypatch.setattr(cli_module, "_RUN_SETTLE_S", 0.0)

        state = cli_module._build_state(_bundled_config_path(), AegisMode.HARDWARE)
        cli_module._run_impl(state, block=None)
        assert "manor.common.aegis.run.run_metis" in spawned
        assert "manor.common.aegis.run.run_kylos" in spawned
        assert "manor.common.aegis.run.run_helios" in spawned
        assert "manor.common.aegis.run.run_gylos" not in spawned

    def test_run_impl_refuses_block_outside_pinned_mode(
        self,
        sandboxed_pid_dir: Path,
    ) -> None:
        state = cli_module._build_state(_bundled_config_path(), mode_override=None)
        with pytest.raises(typer.Exit):
            cli_module._run_impl(state, block=AegisBlock.KYLOS)

    def test_status_impl_in_hardware_mode_lists_hardware_triad(
        self,
        sandboxed_pid_dir: Path,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        state = cli_module._build_state(_bundled_config_path(), AegisMode.HARDWARE)
        cli_module._status_impl(state, block=None)
        out = capsys.readouterr().out
        assert "mode: hardware" in out
        assert "metis: stopped" in out
        assert "kylos: stopped" in out
        assert "helios: stopped" in out
        assert "gylos" not in out


class TestReplLineParser:
    """
    The REPL has its own tiny argv parser (``_parse_repl_block_arg``)
    since it doesn't re-dispatch through click.
    """

    def test_no_block_returns_none(self) -> None:
        assert cli_module._parse_repl_block_arg(["status"]) is None

    def test_known_block_returns_enum(self) -> None:
        assert cli_module._parse_repl_block_arg(["run", "metis"]) is AegisBlock.METIS

    def test_unknown_block_raises(self) -> None:
        with pytest.raises(typer.BadParameter, match="unknown block"):
            cli_module._parse_repl_block_arg(["run", "purple"])

    def test_too_many_args_raises(self) -> None:
        with pytest.raises(typer.BadParameter, match="too many arguments"):
            cli_module._parse_repl_block_arg(["run", "metis", "extra"])


class TestReplHelp:
    """
    The REPL grew its own help support since it no longer goes
    through click. ``-h`` / ``--help`` on its own re-prints the
    top-level help; on a command, it prints that command's help.
    """

    def _state(self) -> "cli_module._CliState":
        return cli_module._build_state(_bundled_config_path(), mode_override=None)

    @pytest.mark.parametrize("flag", ["-h", "--help"])
    def test_bare_help_flag_prints_command_list(
        self,
        flag: str,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        cli_module._dispatch_repl_line(flag, self._state())
        out = capsys.readouterr().out
        assert "commands:" in out
        assert "run [block]" in out
        assert "kill [block]" in out
        assert "status [block]" in out

    @pytest.mark.parametrize("cmd", ["run", "kill", "status"])
    @pytest.mark.parametrize("flag", ["-h", "--help"])
    def test_per_command_help_flag_uses_click_help(
        self,
        cmd: str,
        flag: str,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        # Per-command help in the REPL should be the same click-rendered
        # block you get from ``aegis <cmd> -h`` outside the REPL --
        # not a hand-rolled string.
        cli_module._dispatch_repl_line(f"{cmd} {flag}", self._state())
        out = capsys.readouterr().out
        assert f"Usage: aegis {cmd}" in out
        # The top-level REPL help banner should NOT appear: this is a
        # per-command help, not the command list.
        assert "commands:" not in out

    def test_help_flag_after_block_still_short_circuits(
        self,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        # ``run metis -h`` should print run's click help, not try to
        # spawn.
        cli_module._dispatch_repl_line("run metis -h", self._state())
        out = capsys.readouterr().out
        assert "Usage: aegis run" in out
        assert "started" not in out


if __name__ == "__main__":
    run_manor_tests()
