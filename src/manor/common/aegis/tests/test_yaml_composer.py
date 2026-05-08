"""
Tests for the aegis base / policy / controller YAML composition.

The aegis stack pins per-manipulator base YAMLs (e.g.
``lite6_ac.yaml``) that name a policy and controller via
``policy_type`` / ``controller_type``; the matching bodies live in
``policies/<name>_ac.yaml`` and ``controllers/<name>_ac.yaml``. The
composer in ``aegis.compose_aegis_yaml_dict`` is the single place
that resolves the references and refuses to run aegis when a
sub-YAML is missing or malformed. These tests cover that contract.

The on-disk-coverage tests at the bottom enumerate every member of
``MetisPolicyType`` / ``KyberControllerType`` and assert that the
matching sub-YAML exists and parses through the composer + AegisConfig
parser. New policies / controllers added to the enum without a
matching YAML will fail this layer rather than at run-time.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from manor.common.aegis.aegis import (
    AEGIS_YAML_SUFFIX,
    CONTROLLERS_SUBDIR,
    POLICIES_SUBDIR,
    AegisConfig,
    compose_aegis_yaml_dict,
)
from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerType
from manor.common.aegis.metis.policies.policy_manager import MetisPolicyType
from manor.common.exceptions import AegisConfigError
from manor.common.path_utils import get_project_root
from manor.common.testing_utils import run_manor_tests


def _repo_root() -> Path:
    return Path(get_project_root())


def _bundled_aegis_dir() -> Path:
    return _repo_root() / "configs" / "aegis"


def _bundled_lite6_path() -> Path:
    return _bundled_aegis_dir() / f"lite6{AEGIS_YAML_SUFFIX}"


@pytest.fixture
def aegis_yaml_workspace(tmp_path: Path) -> Path:
    """
    Build a self-contained aegis YAML tree under tmp_path with every
    block needed by the composer. Returns the base YAML path. Tests
    mutate the tree (delete sub-YAMLs, add bogus keys, etc.) to
    exercise the composer's failure paths in isolation.
    """
    policies_dir = tmp_path / POLICIES_SUBDIR
    controllers_dir = tmp_path / CONTROLLERS_SUBDIR
    policies_dir.mkdir()
    controllers_dir.mkdir()

    base = {
        "mode": "sim",
        "manipulator_model": {"type": "lite6", "variant": "parallel_gripper_normal"},
        "environment_config": {},
        "helios_config": {},
        "talos_config": {
            "sim_backend_config": {"minimum_watchdog_frequency_hz": 3.0},
            "hardware_backend_config": {
                "minimum_watchdog_frequency_hz": 3.0,
                "lite6_driver_config": {"joint_speed_limit_rad_s": 1.0},
            },
        },
        "metis_config": {
            "publish_frequency_hz": 10.0,
            "policy_type": "constant_joint_positions",
        },
        "kyber_config": {
            "publish_frequency_hz": 500.0,
            "controller_type": "passthrough",
        },
        "gaia_config": {},
        "gaia_advancer_config": {},
    }
    base_path = tmp_path / f"lite6{AEGIS_YAML_SUFFIX}"
    with open(base_path, "w") as fp:
        yaml.safe_dump(base, fp)

    policy_body = {"positions": [0.0, 0.1, 0.2, 0.0, 0.3, 0.0]}
    with open(policies_dir / f"constant_joint_positions{AEGIS_YAML_SUFFIX}", "w") as fp:
        yaml.safe_dump(policy_body, fp)

    controller_body = {"num_dof": 6}
    with open(controllers_dir / f"passthrough{AEGIS_YAML_SUFFIX}", "w") as fp:
        yaml.safe_dump(controller_body, fp)

    return base_path


class TestComposerHappyPath:
    def test_inlined_dict_parses_through_aegis_config(self, aegis_yaml_workspace: Path) -> None:
        raw = compose_aegis_yaml_dict(aegis_yaml_workspace)
        config = AegisConfig.from_yaml_dict(raw)
        assert config.metis_config.policy_config.POLICY_TYPE is MetisPolicyType.CONSTANT_JOINT_POSITIONS
        assert config.kyber_config.controller_config.CONTROLLER_TYPE is KyberControllerType.PASSTHROUGH

    def test_inlined_block_carries_type_tag(self, aegis_yaml_workspace: Path) -> None:
        # The composer re-injects ``type:`` after stripping it from
        # the sub-YAML body so the existing manager dispatch path
        # (which keys off ``type:``) keeps working unchanged.
        raw = compose_aegis_yaml_dict(aegis_yaml_workspace)
        assert raw["metis_config"]["policy_config"]["type"] == "constant_joint_positions"
        assert raw["kyber_config"]["controller_config"]["type"] == "passthrough"
        assert "policy_type" not in raw["metis_config"]
        assert "controller_type" not in raw["kyber_config"]


class TestComposerRejections:
    def test_rejects_missing_policy_type(self, aegis_yaml_workspace: Path) -> None:
        with open(aegis_yaml_workspace, "r") as fp:
            base = yaml.safe_load(fp)
        del base["metis_config"]["policy_type"]
        with open(aegis_yaml_workspace, "w") as fp:
            yaml.safe_dump(base, fp)
        with pytest.raises(AegisConfigError, match="policy_type"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_missing_controller_type(self, aegis_yaml_workspace: Path) -> None:
        with open(aegis_yaml_workspace, "r") as fp:
            base = yaml.safe_load(fp)
        del base["kyber_config"]["controller_type"]
        with open(aegis_yaml_workspace, "w") as fp:
            yaml.safe_dump(base, fp)
        with pytest.raises(AegisConfigError, match="controller_type"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_unknown_policy_type(self, aegis_yaml_workspace: Path) -> None:
        with open(aegis_yaml_workspace, "r") as fp:
            base = yaml.safe_load(fp)
        base["metis_config"]["policy_type"] = "not_a_real_policy"
        with open(aegis_yaml_workspace, "w") as fp:
            yaml.safe_dump(base, fp)
        with pytest.raises(AegisConfigError, match="not_a_real_policy"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_unknown_controller_type(self, aegis_yaml_workspace: Path) -> None:
        with open(aegis_yaml_workspace, "r") as fp:
            base = yaml.safe_load(fp)
        base["kyber_config"]["controller_type"] = "not_a_real_controller"
        with open(aegis_yaml_workspace, "w") as fp:
            yaml.safe_dump(base, fp)
        with pytest.raises(AegisConfigError, match="not_a_real_controller"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_missing_policy_yaml_file(self, aegis_yaml_workspace: Path) -> None:
        # Sub-YAML existence is the key invariant of the new layout:
        # aegis must refuse to run if the named policy / controller
        # has no body on disk.
        sub_path = aegis_yaml_workspace.parent / POLICIES_SUBDIR / f"constant_joint_positions{AEGIS_YAML_SUFFIX}"
        sub_path.unlink()
        with pytest.raises(AegisConfigError, match="missing"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_missing_controller_yaml_file(self, aegis_yaml_workspace: Path) -> None:
        sub_path = aegis_yaml_workspace.parent / CONTROLLERS_SUBDIR / f"passthrough{AEGIS_YAML_SUFFIX}"
        sub_path.unlink()
        with pytest.raises(AegisConfigError, match="missing"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_type_key_inside_policy_yaml(self, aegis_yaml_workspace: Path) -> None:
        # The type is derived from the filename; embedding ``type:``
        # in the body would let two sources of truth disagree.
        sub_path = aegis_yaml_workspace.parent / POLICIES_SUBDIR / f"constant_joint_positions{AEGIS_YAML_SUFFIX}"
        with open(sub_path, "r") as fp:
            body = yaml.safe_load(fp)
        body["type"] = "constant_joint_positions"
        with open(sub_path, "w") as fp:
            yaml.safe_dump(body, fp)
        with pytest.raises(AegisConfigError, match="type"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_type_key_inside_controller_yaml(self, aegis_yaml_workspace: Path) -> None:
        sub_path = aegis_yaml_workspace.parent / CONTROLLERS_SUBDIR / f"passthrough{AEGIS_YAML_SUFFIX}"
        with open(sub_path, "r") as fp:
            body = yaml.safe_load(fp)
        body["type"] = "passthrough"
        with open(sub_path, "w") as fp:
            yaml.safe_dump(body, fp)
        with pytest.raises(AegisConfigError, match="type"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_inlined_policy_config_in_base(self, aegis_yaml_workspace: Path) -> None:
        # The base must not pre-inline a ``policy_config`` block;
        # the only legal source for a policy body is its sub-YAML.
        with open(aegis_yaml_workspace, "r") as fp:
            base = yaml.safe_load(fp)
        base["metis_config"]["policy_config"] = {"type": "constant_joint_positions", "positions": [0.0] * 6}
        with open(aegis_yaml_workspace, "w") as fp:
            yaml.safe_dump(base, fp)
        with pytest.raises(AegisConfigError, match="policy_config"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)

    def test_rejects_inlined_controller_config_in_base(self, aegis_yaml_workspace: Path) -> None:
        with open(aegis_yaml_workspace, "r") as fp:
            base = yaml.safe_load(fp)
        base["kyber_config"]["controller_config"] = {"type": "passthrough", "num_dof": 6}
        with open(aegis_yaml_workspace, "w") as fp:
            yaml.safe_dump(base, fp)
        with pytest.raises(AegisConfigError, match="controller_config"):
            compose_aegis_yaml_dict(aegis_yaml_workspace)


class TestBundledYamlCoverage:
    """
    Every ``MetisPolicyType`` / ``KyberControllerType`` enum value
    must have a matching ``<name>_ac.yaml`` on disk under the
    bundled ``configs/aegis/policies/`` and
    ``configs/aegis/controllers/`` directories. This is the contract
    the composer enforces at run-time; this layer asserts it at the
    repo level so a new enum value without a YAML is caught in CI.
    """

    @pytest.mark.parametrize("policy_type", list(MetisPolicyType))
    def test_policy_yaml_exists_for_every_enum_value(self, policy_type: MetisPolicyType) -> None:
        path = _bundled_aegis_dir() / POLICIES_SUBDIR / f"{policy_type.value}{AEGIS_YAML_SUFFIX}"
        assert path.exists(), f"missing policy YAML at {path}"

    @pytest.mark.parametrize("controller_type", list(KyberControllerType))
    def test_controller_yaml_exists_for_every_enum_value(self, controller_type: KyberControllerType) -> None:
        path = _bundled_aegis_dir() / CONTROLLERS_SUBDIR / f"{controller_type.value}{AEGIS_YAML_SUFFIX}"
        assert path.exists(), f"missing controller YAML at {path}"

    def test_no_orphan_policy_yamls(self) -> None:
        # The reverse direction: a YAML in ``policies/`` whose name
        # doesn't correspond to an enum value would silently never
        # be referenced. Catch that too so the directory stays a
        # clean source of truth.
        valid = {f"{t.value}{AEGIS_YAML_SUFFIX}" for t in MetisPolicyType}
        on_disk = {p.name for p in (_bundled_aegis_dir() / POLICIES_SUBDIR).glob(f"*{AEGIS_YAML_SUFFIX}")}
        assert on_disk == valid, f"policies/ contents drift: extra={on_disk - valid}, missing={valid - on_disk}"

    def test_no_orphan_controller_yamls(self) -> None:
        valid = {f"{t.value}{AEGIS_YAML_SUFFIX}" for t in KyberControllerType}
        on_disk = {p.name for p in (_bundled_aegis_dir() / CONTROLLERS_SUBDIR).glob(f"*{AEGIS_YAML_SUFFIX}")}
        assert on_disk == valid, f"controllers/ contents drift: extra={on_disk - valid}, missing={valid - on_disk}"


class TestBundledLite6Composition:
    """
    The bundled ``lite6_ac.yaml`` itself must compose into a fully
    parsed ``AegisConfig``. ``test_aegis.py`` already covers the
    diagram-build smoke test against this YAML; this test isolates
    the composer step so a sub-YAML format break surfaces before the
    Drake-heavy build.
    """

    def test_bundled_lite6_yaml_composes_and_parses(self) -> None:
        path = _bundled_lite6_path()
        assert path.exists()
        raw = compose_aegis_yaml_dict(path)
        config = AegisConfig.from_yaml_dict(raw)
        assert config.metis_config.policy_config is not None
        assert config.kyber_config.controller_config is not None


if __name__ == "__main__":
    run_manor_tests()
