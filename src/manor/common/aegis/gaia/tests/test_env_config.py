"""
Tests for EnvironmentConfig + StaticModelConfig YAML loading.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from manor.common.aegis.gaia.env_config import EnvironmentConfig, StaticModelConfig
from manor.common.exceptions import AegisConfigError
from manor.common.path_utils import create_temporary_file
from manor.common.testing_utils import run_manor_tests


def _write(tmp: str, contents: str) -> None:
    with open(tmp, "w") as fp:
        fp.write(contents)


class TestDefault:
    def test_default_is_origin_no_extras(self) -> None:
        cfg = EnvironmentConfig.default()
        np.testing.assert_array_equal(cfg.manipulator_base_xyz, np.zeros(3))
        np.testing.assert_array_equal(cfg.manipulator_base_rpy, np.zeros(3))
        assert cfg.extra_models == ()


class TestFromYaml:
    def test_empty_yaml_yields_default(self) -> None:
        with create_temporary_file(suffix=".yaml") as path:
            _write(path, "")
            cfg = EnvironmentConfig.from_yaml(path)
            np.testing.assert_array_equal(cfg.manipulator_base_xyz, np.zeros(3))
            assert cfg.extra_models == ()

    def test_loads_manipulator_pose(self) -> None:
        with create_temporary_file(suffix=".yaml") as path:
            _write(
                path,
                "manipulator_base_xyz: [0.1, 0.2, 0.3]\nmanipulator_base_rpy: [0.0, 0.0, 1.5707]\n",
            )
            cfg = EnvironmentConfig.from_yaml(path)
            np.testing.assert_allclose(cfg.manipulator_base_xyz, [0.1, 0.2, 0.3])
            np.testing.assert_allclose(cfg.manipulator_base_rpy, [0.0, 0.0, 1.5707])

    def test_loads_extra_models_with_absolute_path(self) -> None:
        from manor.common.model_utils import get_models_directory_path

        absolute_urdf = os.path.join(get_models_directory_path(), "environment", "lite6_table.urdf")
        with create_temporary_file(suffix=".yaml") as path:
            _write(
                path,
                "extra_models:\n"
                "  - name: table\n"
                f"    description_filepath: {absolute_urdf}\n"
                "    weld_to_world: false\n",
            )
            cfg = EnvironmentConfig.from_yaml(path)
            assert len(cfg.extra_models) == 1
            assert isinstance(cfg.extra_models[0], StaticModelConfig)
            assert cfg.extra_models[0].description_filepath == absolute_urdf

    def test_relative_path_resolves_against_models_directory(self) -> None:
        from manor.common.model_utils import get_models_directory_path

        with create_temporary_file(suffix=".yaml") as path:
            _write(
                path,
                "extra_models:\n  - name: table\n    description_filepath: environment/lite6_table.urdf\n",
            )
            cfg = EnvironmentConfig.from_yaml(path)
            expected = os.path.normpath(os.path.join(get_models_directory_path(), "environment/lite6_table.urdf"))
            assert cfg.extra_models[0].description_filepath == expected

    def test_invalid_top_level_raises(self) -> None:
        with create_temporary_file(suffix=".yaml") as path:
            _write(path, "- not\n- a\n- mapping\n")
            with pytest.raises(AegisConfigError):
                EnvironmentConfig.from_yaml(path)

    def test_missing_required_static_model_field_raises(self) -> None:
        with create_temporary_file(suffix=".yaml") as path:
            _write(path, "extra_models:\n  - description_filepath: /tmp/x.urdf\n")
            with pytest.raises(AegisConfigError):
                EnvironmentConfig.from_yaml(path)

    def test_bad_xyz_length_raises(self) -> None:
        with create_temporary_file(suffix=".yaml") as path:
            _write(path, "manipulator_base_xyz: [1.0, 2.0]\n")
            with pytest.raises(AegisConfigError):
                EnvironmentConfig.from_yaml(path)


class TestBundledLite6TableYaml:
    def test_loads_default_lite6_table_env(self) -> None:
        """
        The bundled lite6_table.yaml under models/aegis_envs must load
        cleanly with paths resolved to the actual on-disk URDF.
        """
        from manor.common.model_utils import get_models_directory_path

        path = os.path.join(get_models_directory_path(), "aegis_envs", "lite6_table.yaml")
        assert os.path.exists(path), f"bundled env yaml missing at {path}"
        cfg = EnvironmentConfig.from_yaml(path)
        assert len(cfg.extra_models) == 1
        table = cfg.extra_models[0]
        assert table.name == "lite6_table"
        assert os.path.exists(table.description_filepath), (
            f"resolved description_filepath does not exist: {table.description_filepath}"
        )
        assert table.weld_to_world is False


if __name__ == "__main__":
    run_manor_tests()
