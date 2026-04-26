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

import pytest
from click.testing import CliRunner

from manor.common.aegis import aegis_cli as cli_module
from manor.common.aegis.aegis_cli import AegisBlock, cli
from manor.common.testing_utils import run_manor_tests


def _bundled_config_path() -> Path:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    return Path(repo_root) / "configs" / "aegis" / "lite6_default.yaml"


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
    return runner.invoke(cli, ["--config", str(_bundled_config_path()), *args])


class TestAegisCli:
    def test_list_reports_sim_blocks(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["list"])
        assert result.exit_code == 0, result.output
        assert "mode: sim" in result.output
        assert "metis: stopped" in result.output
        assert "gylos: stopped" in result.output

    def test_status_unknown_block_rejected(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status", "not_a_block"])
        # typer's enum coercion produces a non-zero exit on bad values.
        assert result.exit_code != 0

    def test_kill_without_running_block_errors(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["kill", "metis"])
        assert result.exit_code != 0
        assert "is not running" in result.output

    def test_run_refuses_hardware_block_in_sim_mode(self, sandboxed_pid_dir: Path) -> None:
        # The bundled config is sim mode; kylos is hardware-only.
        result = _run_cli(["run", "kylos"])
        assert result.exit_code != 0
        assert "refusing to run" in result.output

    def test_status_requires_block_argument(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status"])
        # ``status`` now requires <block>; no fallback to listing.
        assert result.exit_code != 0

    def test_status_reports_stopped_for_known_block(self, sandboxed_pid_dir: Path) -> None:
        result = _run_cli(["status", "metis"])
        assert result.exit_code == 0
        assert "metis" in result.output
        assert "stopped" in result.output

    def test_pid_file_path_is_block_specific(self, sandboxed_pid_dir: Path) -> None:
        path = cli_module._pid_file_path(AegisBlock.METIS)
        assert path.parent == sandboxed_pid_dir
        assert path.name == "aegis_metis.pid"


if __name__ == "__main__":
    run_manor_tests()
