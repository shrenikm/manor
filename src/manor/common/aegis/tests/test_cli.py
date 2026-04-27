"""
Tests for the ``aegis`` CLI / REPL.

We don't drive the prompt_toolkit REPL from inside pytest -- instead
we exercise the typer commands directly via click's ``CliRunner``.
PID-file state is sandboxed by pointing ``_PID_FILE_DIR`` at a
``tmp_path`` for each test.
"""

from __future__ import annotations

import os
from pathlib import Path
from unittest import mock

import pytest
from click.testing import CliRunner

from manor.common.aegis import aegis_cli as cli_module
from manor.common.aegis.aegis_cli import AegisBlock, cli
from manor.common.testing_utils import run_manor_tests


def _bundled_config_path() -> Path:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    return Path(repo_root) / "configs" / "aegis" / "default_ac.yaml"


class _FakeStdin:
    """
    Minimal stand-in for the Popen child's stdin that ``_spawn_block``
    writes the JSON config payload into. Real ``Popen`` returns a
    ``BufferedWriter``; tests just need the two methods we touch.
    """

    def write(self, _data: bytes) -> int:
        return 0

    def close(self) -> None:
        return None


@pytest.fixture
def sandboxed_pid_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """
    Redirect PID files into a per-test tmp directory so concurrent
    test runs (and stray processes from a previous local run) don't
    cross-contaminate the assertions.
    """
    monkeypatch.setattr(cli_module, "_PID_FILE_DIR", tmp_path)
    return tmp_path


def _run_cli(args: list[str], inject_config: bool = True) -> "CliRunner.Result":
    """
    Invoke the CLI with the bundled default config injected onto
    ``run`` / ``status`` / ``repl`` lines (the commands that accept
    ``--config``). ``kill`` is PID-only so its argv is passed through
    untouched.

    Set ``inject_config=False`` to exercise the default-path resolution
    (the bundled config is already the default, so they're equivalent
    for assertions but tests of explicit overrides skip injection).
    """
    runner = CliRunner()
    needs_config = bool(args) and args[0] in {"run", "status", "repl"} and inject_config
    if needs_config:
        full = [args[0], "--config", str(_bundled_config_path()), *args[1:]]
    else:
        full = list(args)
    return runner.invoke(cli, full)


class TestAegisCli:
    def test_status_no_arg_labels_blocks_by_mode_applicability(
        self,
        sandboxed_pid_dir: Path,
    ) -> None:
        # ``status`` no-arg lists every AegisBlock and labels each
        # one. In sim mode, metis + gylos are applicable (so
        # ``stopped`` when nothing's running), kylos + helios are
        # ``unavailable``.
        result = _run_cli(["status"])
        assert result.exit_code == 0, result.output
        assert "metis: stopped" in result.output
        assert "gylos: stopped" in result.output
        assert "kylos: unavailable" in result.output
        assert "helios: unavailable" in result.output

    def test_status_marks_non_applicable_block_unavailable_for_single_query(
        self,
        sandboxed_pid_dir: Path,
    ) -> None:
        # ``status kylos`` in sim mode should say ``unavailable``,
        # not ``stopped`` -- you can't run kylos here at all.
        result = _run_cli(["status", "kylos"])
        assert result.exit_code == 0, result.output
        assert "kylos: unavailable" in result.output

    def test_status_mode_override_flips_unavailable_set(
        self,
        sandboxed_pid_dir: Path,
    ) -> None:
        # ``-m hardware`` should make gylos unavailable and kylos /
        # helios available (stopped, since nothing is running).
        result = _run_cli(["status", "-m", "hardware"])
        assert result.exit_code == 0, result.output
        assert "metis: stopped" in result.output
        assert "gylos: unavailable" in result.output
        assert "kylos: stopped" in result.output
        assert "helios: stopped" in result.output

    def test_status_unknown_block_rejected(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status", "not_a_block"])
        # typer's enum coercion produces a non-zero exit on bad values.
        assert result.exit_code != 0

    def test_kill_without_running_block_errors(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["kill", "metis"])
        assert result.exit_code != 0
        assert "is not running" in result.output

    def test_kill_no_arg_with_nothing_running_is_noop(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["kill"])
        assert result.exit_code == 0
        assert "nothing running" in result.output

    def test_run_refuses_hardware_block_in_sim_mode(self, sandboxed_pid_dir: Path) -> None:
        # The bundled config is sim mode; kylos is hardware-only.
        result = _run_cli(["run", "kylos"])
        assert result.exit_code != 0
        assert "refusing to run" in result.output

    def test_status_reports_stopped_for_known_block(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status", "metis"])
        assert result.exit_code == 0
        assert "metis" in result.output
        assert "stopped" in result.output

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
            # args[2] is the ``-m <module>`` target.
            spawned.append(args[2])
            proc = mock.MagicMock()
            proc.pid = 90000 + len(spawned)
            return proc

        monkeypatch.setattr(cli_module.subprocess, "Popen", fake_popen)
        monkeypatch.setattr(cli_module, "_RUN_SETTLE_S", 0.0)

        result = _run_cli(["run"])
        assert result.exit_code == 0, result.output
        # Sim mode: every applicable block (metis, gylos) gets spawned.
        assert "manor.common.aegis.run.run_metis" in spawned
        assert "manor.common.aegis.run.run_gylos" in spawned
        assert "started metis" in result.output
        assert "started gylos" in result.output

    def test_run_no_arg_with_all_blocks_running_is_noop(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # Pretend every applicable block already has a live PID.
        monkeypatch.setattr(cli_module, "_process_alive", lambda _pid: True)
        for block in (AegisBlock.METIS, AegisBlock.GYLOS):
            cli_module._write_pid(block, 12345)

        result = _run_cli(["run"])
        assert result.exit_code == 0
        assert "already running" in result.output
        assert "nothing to start" in result.output

    def test_mode_override_flips_applicable_blocks(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # The bundled config is sim mode; ``--mode hardware`` should
        # coerce it so kylos / helios are the applicable blocks and
        # gylos is now refused.
        spawned: list[str] = []

        def fake_popen(args: list[str], **_kwargs: object) -> mock.MagicMock:
            spawned.append(args[2])
            proc = mock.MagicMock()
            proc.pid = 80000 + len(spawned)
            return proc

        monkeypatch.setattr(cli_module.subprocess, "Popen", fake_popen)
        monkeypatch.setattr(cli_module, "_RUN_SETTLE_S", 0.0)

        result = _run_cli(["run", "--mode", "hardware"])
        assert result.exit_code == 0, result.output
        # Hardware-mode triad: metis, kylos, helios (no gylos).
        assert "manor.common.aegis.run.run_metis" in spawned
        assert "manor.common.aegis.run.run_kylos" in spawned
        assert "manor.common.aegis.run.run_helios" in spawned
        assert "manor.common.aegis.run.run_gylos" not in spawned

    def test_mode_override_short_flag_refuses_sim_only_block(
        self,
        sandboxed_pid_dir: Path,
    ) -> None:
        # ``-m hardware`` on a sim config should make gylos invalid.
        result = _run_cli(["run", "gylos", "-m", "hardware"])
        assert result.exit_code != 0
        assert "refusing to run" in result.output

    def test_kill_does_not_accept_config_flag(self, sandboxed_pid_dir: Path) -> None:
        # ``kill`` and ``status`` are PID-only -- they don't take
        # ``--config`` or ``--mode``. Passing ``--config`` should be
        # rejected by click as an unknown option.
        runner = CliRunner()
        result = runner.invoke(cli, ["kill", "--config", str(_bundled_config_path())])
        assert result.exit_code != 0

    def test_config_bare_filename_resolves_under_configs_dir(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # Bare filename should be looked up in configs/aegis/. The
        # bundled default lives there, so passing just the filename
        # is a working invocation.
        monkeypatch.setattr(cli_module, "_RUN_SETTLE_S", 0.0)
        monkeypatch.setattr(
            cli_module.subprocess,
            "Popen",
            lambda *_a, **_k: type("P", (), {"pid": 70001, "stdin": _FakeStdin()})(),
        )
        runner = CliRunner()
        result = runner.invoke(cli, ["run", "metis", "-c", "default_ac.yaml"])
        assert result.exit_code == 0, result.output
        assert "started metis" in result.output

    def test_config_missing_filename_errors(self, sandboxed_pid_dir: Path) -> None:
        # Non-existent filename under configs/aegis/ should fail fast
        # rather than silently fall back to the default. This is the
        # case that motivated the resolution rule.
        runner = CliRunner()
        result = runner.invoke(cli, ["run", "metis", "-c", "does_not_exist_ac.yaml"])
        assert result.exit_code != 0
        assert "config file not found" in result.output


if __name__ == "__main__":
    run_manor_tests()
