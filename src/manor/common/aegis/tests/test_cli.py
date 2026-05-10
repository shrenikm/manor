"""
Tests for the aegis CLI / REPL.

The standalone CLI commands (aegis run / aegis kill / aegis status) are exercised via click's CliRunner. The REPL
itself isn't driven from pytest -- instead we test the impl functions (_run_impl / _status_impl / _kill_impl) and the
parser / state builder it composes, so the REPL's behaviour is covered without spinning up prompt_toolkit.

PID-file state is sandboxed by pointing _PID_FILE_DIR at a tmp_path for each test.
"""

from __future__ import annotations

import os
import shutil
from pathlib import Path
from unittest import mock

import pytest
import typer
import yaml
from click.testing import CliRunner

from manor.common.aegis import aegis_cli as cli_module
from manor.common.aegis.aegis_cli import AegisBlock, cli
from manor.common.aegis.mode import AegisMode
from manor.common.testing_utils import run_manor_tests


def _bundled_config_path() -> Path:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    return Path(repo_root) / "configs" / "aegis" / "lite6_ac.yaml"


@pytest.fixture
def sandboxed_pid_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """
    Redirect PID files into a per-test tmp directory so concurrent
    test runs (and stray processes from a previous local run) don't
    cross-contaminate the assertions.
    """
    monkeypatch.setattr(cli_module, "_PID_FILE_DIR", tmp_path)
    return tmp_path


@pytest.fixture(autouse=True)
def stub_find_orphan_pids(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    /proc is global state; an unrelated process on the dev machine
    matching a runner module's argv would otherwise leak into kill /
    status assertions. Default tests to "no orphans". Tests that want
    to exercise orphan handling can override this with monkeypatch.
    """
    monkeypatch.setattr(cli_module, "_find_orphan_pids", lambda _block: [])


@pytest.fixture
def sim_default_config(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """
    Mirror the bundled aegis configs into tmp_path with the base YAML's
    mode forced to sim, and point cli_module._DEFAULT_CONFIG_PATH at the
    copy. The standalone CLI commands take no --mode flag, so the only
    way to exercise sim-mode behaviour end-to-end is to pin a sim-mode
    YAML in place of the bundled default. Decoupling the tests from the
    bundled YAML's current mode also keeps them stable across local edits.
    """
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    src_dir = Path(repo_root) / "configs" / "aegis"
    dst_dir = tmp_path / "aegis"
    shutil.copytree(src_dir, dst_dir)
    yaml_path = dst_dir / "lite6_ac.yaml"
    with open(yaml_path, "r") as fp:
        base = yaml.safe_load(fp)
    base["mode"] = AegisMode.SIM.value
    with open(yaml_path, "w") as fp:
        yaml.safe_dump(base, fp)
    monkeypatch.setattr(cli_module, "_DEFAULT_CONFIG_PATH", yaml_path)
    return yaml_path


def _run_cli(args: list[str]) -> "CliRunner.Result":
    runner = CliRunner()
    return runner.invoke(cli, args)


class TestStandaloneCli:
    """
    The non-REPL commands always read the bundled default config; no
    overrides are accepted. The bundled default is sim mode.
    """

    def test_status_no_arg_lists_only_applicable_blocks(
        self,
        sandboxed_pid_dir: Path,
        sim_default_config: Path,
    ) -> None:
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

    def test_run_refuses_hardware_block_in_sim_mode(
        self,
        sandboxed_pid_dir: Path,
        sim_default_config: Path,
    ) -> None:
        # Pinned sim-mode YAML; kylos is hardware-only.
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
        sim_default_config: Path,
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
        sim_default_config: Path,
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
        # --config / --mode are REPL-only. Outside the REPL the YAML is the single source of truth; passing override
        # flags must fail at parse time so users get a clear error instead of silent fallback.
        result = _run_cli([command, flag, "anything"])
        assert result.exit_code != 0


class TestConfigResolution:
    """
    The --config resolution helper used by aegis repl -c .... Standalone commands don't accept the flag, but the
    resolver is still public-ish and worth covering directly.
    """

    def test_bare_filename_resolves_under_configs_dir(self) -> None:
        resolved = cli_module._resolve_config_path(Path("lite6_ac.yaml"))
        assert resolved == _bundled_config_path()

    def test_absolute_path_passes_through(self) -> None:
        abs_path = _bundled_config_path()
        assert cli_module._resolve_config_path(abs_path) == abs_path

    def test_missing_file_errors(self) -> None:
        with pytest.raises(typer.BadParameter, match="config file not found"):
            cli_module._load_config(Path("does_not_exist_ac.yaml"), mode_override=None)


class TestReplPinnedState:
    """
    The REPL is wired around _build_state (called once at REPL launch) plus the _*_impl functions (called per line).
    Driving prompt_toolkit from pytest is hostile, so we cover the moving parts directly with the pinned state.
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
        # Pin sim mode so kylos (hardware-only) is unambiguously out of scope.
        state = cli_module._build_state(_bundled_config_path(), mode_override=AegisMode.SIM)
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
    The REPL has its own tiny argv parser (_parse_repl_block_arg) since it doesn't re-dispatch through click.
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
    The REPL grew its own help support since it no longer goes through click. -h / --help on its own re-prints the
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
        # Per-command help in the REPL should be the same click-rendered block you get from aegis <cmd> -h outside
        # the REPL -- not a hand-rolled string.
        cli_module._dispatch_repl_line(f"{cmd} {flag}", self._state())
        out = capsys.readouterr().out
        assert f"Usage: aegis {cmd}" in out
        # The top-level REPL help banner should NOT appear: this is a
        # per-command help, not the command list.
        assert "commands:" not in out

    @pytest.mark.parametrize("flag", ["-h", "--help"])
    def test_reload_help_flag_prints_reload_help(
        self,
        flag: str,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        # reload is REPL-only (no click counterpart) so its per-command help is hand-rolled rather than rendered by
        # click. Verify the hand-rolled banner shows up.
        state = cli_module._build_state(_bundled_config_path(), mode_override=None)
        cli_module._dispatch_repl_line(f"reload {flag}", state)
        out = capsys.readouterr().out
        assert "Usage: aegis reload" in out

    def test_help_flag_after_block_still_short_circuits(
        self,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        # run metis -h should print run's click help, not try to spawn.
        cli_module._dispatch_repl_line("run metis -h", self._state())
        out = capsys.readouterr().out
        assert "Usage: aegis run" in out
        assert "started" not in out


@pytest.fixture
def aegis_yaml_sandbox(tmp_path: Path) -> Path:
    """
    Mirror the bundled configs/aegis/ tree into tmp_path so a test can edit YAMLs (base, policy, controller) without
    touching the real repo. Returns the path to the copied lite6_ac.yaml; the policies/ and controllers/ subdirs come
    along beside it so the composer's sibling-directory resolution still finds them.
    """
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    src_dir = Path(repo_root) / "configs" / "aegis"
    dst_dir = tmp_path / "aegis"
    shutil.copytree(src_dir, dst_dir)
    return dst_dir / "lite6_ac.yaml"


class TestReplReload:
    """
    reload re-reads the YAML configs from disk inside the REPL so the user can iterate on a policy / controller / sim
    setting without restarting the REPL. The launch-time --mode override is preserved across reloads; already-running
    blocks keep their stale config (with a warning) until they're restarted.
    """

    def test_reload_picks_up_metis_publish_frequency_change(
        self,
        aegis_yaml_sandbox: Path,
    ) -> None:
        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        original_hz = state.config.metis_config.publish_frequency_hz

        with open(aegis_yaml_sandbox, "r") as fp:
            base = yaml.safe_load(fp)
        new_hz = original_hz + 5.0
        base["metis_config"]["publish_frequency_hz"] = new_hz
        with open(aegis_yaml_sandbox, "w") as fp:
            yaml.safe_dump(base, fp)

        new_state = cli_module._reload_impl(state)
        assert new_state is not state
        assert new_state.config.metis_config.publish_frequency_hz == new_hz

    def test_reload_picks_up_policy_type_change(
        self,
        aegis_yaml_sandbox: Path,
    ) -> None:
        # The bundled lite6_ac.yaml may swap its policy_type between
        # sessions, so anchor the assertion on a switch we make in
        # the test rather than on the starting value.
        from manor.common.aegis.metis.policies.policy_manager import MetisPolicyType

        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        with open(aegis_yaml_sandbox, "r") as fp:
            base = yaml.safe_load(fp)
        target = (
            MetisPolicyType.IDENTITY
            if base["metis_config"]["policy_type"] != MetisPolicyType.IDENTITY.value
            else MetisPolicyType.CONSTANT_JOINT_POSITIONS
        )
        base["metis_config"]["policy_type"] = target.value
        with open(aegis_yaml_sandbox, "w") as fp:
            yaml.safe_dump(base, fp)

        new_state = cli_module._reload_impl(state)
        assert new_state.config.metis_config.policy_config.POLICY_TYPE is target

    def test_reload_picks_up_policy_sub_yaml_edit(
        self,
        aegis_yaml_sandbox: Path,
    ) -> None:
        # The motivating workflow: edit a policy sub-YAML between two run metis invocations and have reload pick up
        # the change without restarting the REPL.
        from manor.common.aegis.metis.policies.constant_policies import (
            ConstantJointPositionsPolicyConfig,
        )

        with open(aegis_yaml_sandbox, "r") as fp:
            base = yaml.safe_load(fp)
        base["metis_config"]["policy_type"] = "constant_joint_positions"
        with open(aegis_yaml_sandbox, "w") as fp:
            yaml.safe_dump(base, fp)

        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        assert isinstance(state.config.metis_config.policy_config, ConstantJointPositionsPolicyConfig)

        sub_path = aegis_yaml_sandbox.parent / "policies" / "constant_joint_positions_ac.yaml"
        new_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        with open(sub_path, "w") as fp:
            yaml.safe_dump({"positions": new_positions}, fp)

        new_state = cli_module._reload_impl(state)
        assert isinstance(new_state.config.metis_config.policy_config, ConstantJointPositionsPolicyConfig)
        assert list(new_state.config.metis_config.policy_config.positions) == new_positions

    def test_reload_preserves_mode_override(
        self,
        aegis_yaml_sandbox: Path,
    ) -> None:
        # A session started with --mode hardware against a sim YAML must keep the hardware override after reload --
        # otherwise the user's pinned setting silently regresses.
        with open(aegis_yaml_sandbox, "r") as fp:
            base = yaml.safe_load(fp)
        base["mode"] = "sim"
        with open(aegis_yaml_sandbox, "w") as fp:
            yaml.safe_dump(base, fp)

        state = cli_module._build_state(aegis_yaml_sandbox, AegisMode.HARDWARE)
        assert state.config.mode is AegisMode.HARDWARE
        assert state.mode_override is AegisMode.HARDWARE

        new_state = cli_module._reload_impl(state)
        assert new_state.config.mode is AegisMode.HARDWARE
        assert new_state.mode_override is AegisMode.HARDWARE

    def test_reload_warns_when_blocks_are_running(
        self,
        aegis_yaml_sandbox: Path,
        sandboxed_pid_dir: Path,
        capsys: pytest.CaptureFixture[str],
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # reload only affects blocks spawned *after* it; we warn so the user notices when their edit won't take
        # effect on already-running children.
        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        monkeypatch.setattr(cli_module, "_process_alive", lambda _pid: True)
        cli_module._write_pid(AegisBlock.METIS, 12345)

        new_state = cli_module._reload_impl(state)
        captured = capsys.readouterr()
        out = captured.out + captured.err
        assert "metis" in out
        assert "still running" in out
        assert new_state is not state

    def test_reload_keeps_old_state_on_invalid_edit(
        self,
        aegis_yaml_sandbox: Path,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        # If the user botches the YAML, reload keeps the previous state instead of dropping the user out of the REPL.
        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        with open(aegis_yaml_sandbox, "r") as fp:
            base = yaml.safe_load(fp)
        base["metis_config"]["policy_type"] = "not_a_real_policy"
        with open(aegis_yaml_sandbox, "w") as fp:
            yaml.safe_dump(base, fp)

        new_state = cli_module._reload_impl(state)
        captured = capsys.readouterr()
        out = captured.out + captured.err
        assert "reload failed" in out
        assert new_state is state

    def test_reload_via_dispatch_returns_new_state(
        self,
        aegis_yaml_sandbox: Path,
    ) -> None:
        # The REPL loop relies on the dispatcher's return value to re-bind its local state. A reload that returned
        # the old state would silently no-op, so anchor that contract.
        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        with open(aegis_yaml_sandbox, "r") as fp:
            base = yaml.safe_load(fp)
        base["metis_config"]["publish_frequency_hz"] = state.config.metis_config.publish_frequency_hz + 7.5
        with open(aegis_yaml_sandbox, "w") as fp:
            yaml.safe_dump(base, fp)

        returned = cli_module._dispatch_repl_line("reload", state)
        assert returned is not state
        assert returned.config.metis_config.publish_frequency_hz == state.config.metis_config.publish_frequency_hz + 7.5

    def test_reload_rejects_extra_args(
        self,
        aegis_yaml_sandbox: Path,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        state = cli_module._build_state(aegis_yaml_sandbox, mode_override=None)
        returned = cli_module._dispatch_repl_line("reload extra", state)
        captured = capsys.readouterr()
        out = captured.out + captured.err
        assert "takes no arguments" in out
        # Extra-arg failure is a no-op: state is preserved.
        assert returned is state

    def test_reload_appears_in_top_level_help_and_completer(
        self,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        cli_module._print_repl_help()
        out = capsys.readouterr().out
        assert "reload" in out
        completer = cli_module._build_completer()
        assert "reload" in completer.words


class TestOrphanHandling:
    """
    The kill / status paths sweep up runner-module processes that
    aren't tracked by the PID file (e.g. a previous SIGTERM was
    ignored and the PID file got cleared anyway). Cover the matcher
    and the kill-side reaping.
    """

    def test_find_orphan_pids_matches_python_dash_m_invocation(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
        tmp_path: Path,
    ) -> None:
        # _find_orphan_pids should match argv == [python, -m, <module>, ...]
        # exactly. An editor with the runner file open carries the file
        # path positionally (slashes, .py extension), not -m <module>, so
        # it must be ignored.
        # Undo the autouse stub so we exercise the real implementation.
        monkeypatch.undo()
        monkeypatch.setattr(cli_module, "_PID_FILE_DIR", sandboxed_pid_dir)

        proc_root = tmp_path / "proc"
        proc_root.mkdir()
        # Genuine orphan: python -m manor.common.aegis.run.run_helios.
        _write_fake_proc(
            proc_root, pid=1111, argv=["/opt/python", "-m", cli_module._BLOCK_RUN_MODULE[AegisBlock.HELIOS]]
        )
        # Editor with the runner file open: positional path argument, no -m.
        _write_fake_proc(
            proc_root,
            pid=2222,
            argv=["/usr/bin/nvim", "/home/shrenikm/Projects/manor/src/manor/common/aegis/run/run_helios.py"],
        )
        # A different block's runner: must not match HELIOS.
        _write_fake_proc(
            proc_root, pid=3333, argv=["/opt/python", "-m", cli_module._BLOCK_RUN_MODULE[AegisBlock.METIS]]
        )
        # Tracked PID for HELIOS: must be filtered out via the read_pid check.
        cli_module._write_pid(AegisBlock.HELIOS, 4444)
        _write_fake_proc(
            proc_root, pid=4444, argv=["/opt/python", "-m", cli_module._BLOCK_RUN_MODULE[AegisBlock.HELIOS]]
        )
        # Force the tracked-PID check to consider 4444 alive.
        monkeypatch.setattr(cli_module, "_process_alive", lambda pid: pid == 4444)
        monkeypatch.setattr(cli_module, "Path", _PathProxy(real_path_class=Path, proc_root=proc_root))

        orphans = cli_module._find_orphan_pids(AegisBlock.HELIOS)
        assert orphans == [1111]

    def test_kill_block_reaps_orphans_when_pid_file_empty(
        self,
        sandboxed_pid_dir: Path,
        monkeypatch: pytest.MonkeyPatch,
    ) -> None:
        # No tracked PID for helios, but two orphan processes exist.
        # `aegis kill helios` should _terminate both and exit 0
        # (instead of erroring out with "is not running").
        terminated: list[int] = []
        monkeypatch.setattr(
            cli_module, "_find_orphan_pids", lambda block: [7001, 7002] if block is AegisBlock.HELIOS else []
        )
        monkeypatch.setattr(cli_module, "_terminate", lambda pid, _label: terminated.append(pid))

        result = _run_cli(["kill", "helios"])
        assert result.exit_code == 0, result.output
        assert terminated == [7001, 7002]
        assert "reaping orphan helios (pid 7001)" in result.output
        assert "reaping orphan helios (pid 7002)" in result.output

    def test_kill_block_errors_when_no_pid_and_no_orphans(
        self,
        sandboxed_pid_dir: Path,
    ) -> None:
        # Nothing tracked, no orphans (autouse stub returns []) -> error.
        result = _run_cli(["kill", "helios"])
        assert result.exit_code != 0
        assert "is not running" in result.output


def _write_fake_proc(proc_root: Path, pid: int, argv: list[str]) -> None:
    """
    Write a /proc/<pid>/cmdline-shaped fixture (null-separated argv,
    trailing null) under proc_root for _find_orphan_pids to discover.
    """
    pid_dir = proc_root / str(pid)
    pid_dir.mkdir()
    payload = b"\x00".join(s.encode("utf-8") for s in argv) + b"\x00"
    (pid_dir / "cmdline").write_bytes(payload)


class _PathProxy:
    """
    Stand-in for pathlib.Path inside cli_module that redirects
    Path("/proc") to a tmp_path-rooted proc tree but otherwise behaves
    like the real Path class. Lets _find_orphan_pids walk a fake /proc
    without touching the host's /proc.
    """

    def __init__(self, real_path_class: type, proc_root: Path) -> None:
        self._real = real_path_class
        self._proc_root = proc_root

    def __call__(self, *args: object, **kwargs: object) -> Path:
        if len(args) == 1 and args[0] == "/proc":
            return self._proc_root
        return self._real(*args, **kwargs)

    def __getattr__(self, name: str) -> object:
        return getattr(self._real, name)


if __name__ == "__main__":
    run_manor_tests()
