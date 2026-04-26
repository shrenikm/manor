"""
Environment configuration for the aegis simulator.

An ``EnvironmentConfig`` describes how the manipulator is mounted and
which static models populate the world around it. Configurations can be
authored as YAML and loaded through ``EnvironmentConfig.from_yaml``; the
default config (no extras, manipulator welded to the world origin) is
also available via ``EnvironmentConfig.default()``.

Relative ``description_filepath`` values are resolved against the
project's ``models/`` directory at load time, so a YAML can reference
``environment/lite6_table.urdf`` regardless of where the YAML itself
lives. Absolute paths pass through unchanged.

YAML schema (all fields optional):

```yaml
manipulator_base_xyz: [0.0, 0.0, 0.0]
manipulator_base_rpy: [0.0, 0.0, 0.0]
extra_models:
  - name: table
    description_filepath: environment/lite6_table.urdf
    base_xyz: [0.0, 0.0, 0.0]
    base_rpy: [0.0, 0.0, 0.0]
    weld_to_world: true
```
"""

from __future__ import annotations

import os
from typing import Self

import attr
import numpy as np
import yaml

from manor.common.aegis.yaml_utils import assert_keys_match_attrs, require_bool, require_str
from manor.common.custom_types import FilePath, NpVector3f64
from manor.common.exceptions import AegisConfigError
from manor.common.model_utils import get_models_directory_path


def _zero_xyz() -> NpVector3f64:
    return np.zeros(3, dtype=np.float64)


def _zero_rpy() -> NpVector3f64:
    return np.zeros(3, dtype=np.float64)


@attr.frozen
class StaticModelConfig:
    """
    A single static model loaded into the simulation world.

    The model is loaded from ``description_filepath``. If
    ``weld_to_world`` is true, Gaia explicitly welds the model's base
    body to the world frame at the configured pose.

    Set ``weld_to_world`` to false for URDFs / SDFs that already pin
    themselves to the world via an internal fixed joint (common in
    environment URDFs). In that case ``base_xyz`` / ``base_rpy`` are
    informational only -- the model's own internal joint defines the
    final pose.
    """

    name: str
    description_filepath: FilePath
    base_xyz: NpVector3f64 = attr.field(factory=_zero_xyz)
    base_rpy: NpVector3f64 = attr.field(factory=_zero_rpy)
    weld_to_world: bool = True

    @classmethod
    def from_yaml_dict(cls, raw: dict, context: str) -> Self:
        """
        Parse a single ``extra_models[i]`` mapping. ``context`` is
        used for error messages (e.g. ``"extra_models[0]"``).
        """
        assert_keys_match_attrs(cls, raw, context)
        return cls(
            name=require_str(raw.get("name"), f"{context}.name"),
            description_filepath=_resolve_filepath(
                require_str(raw.get("description_filepath"), f"{context}.description_filepath")
            ),
            base_xyz=_parse_xyz(raw.get("base_xyz"), f"{context}.base_xyz"),
            base_rpy=_parse_xyz(raw.get("base_rpy"), f"{context}.base_rpy"),
            weld_to_world=require_bool(raw.get("weld_to_world", True), f"{context}.weld_to_world"),
        )


def _parse_xyz(value: object, field_name: str) -> NpVector3f64:
    if value is None:
        return np.zeros(3, dtype=np.float64)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise AegisConfigError(f"'{field_name}' must be a length-3 list of floats; got {value!r}")
    try:
        return np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as e:
        raise AegisConfigError(f"'{field_name}' could not be coerced to a float vector: {e}") from e


def _resolve_filepath(path: FilePath) -> FilePath:
    """
    Relative ``description_filepath`` values resolve against the
    project's ``models/`` directory; absolute paths pass through
    unchanged. So a YAML can reference ``environment/foo.urdf`` and
    Gaia will find it at ``<project_root>/models/environment/foo.urdf``
    regardless of where the YAML itself sits.
    """
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(get_models_directory_path(), path))


@attr.frozen
class EnvironmentConfig:
    """
    Top-level environment description for the aegis simulator.

    ``manipulator_base_xyz`` / ``manipulator_base_rpy`` give the pose at
    which the manipulator is welded to the world. ``extra_models`` lists
    any additional models to load into the same plant.
    """

    manipulator_base_xyz: NpVector3f64 = attr.field(factory=_zero_xyz)
    manipulator_base_rpy: NpVector3f64 = attr.field(factory=_zero_rpy)
    extra_models: tuple[StaticModelConfig, ...] = attr.field(factory=tuple)

    @classmethod
    def default(cls) -> Self:
        """
        Construct the no-extras default: manipulator welded at the world
        origin with no axis offsets, no extra models.
        """
        return cls()

    @classmethod
    def from_yaml(cls, filepath: FilePath) -> Self:
        """
        Load an EnvironmentConfig from a YAML file.

        Missing fields fall back to the defaults from ``default()``.
        Relative ``description_filepath`` values inside ``extra_models``
        are resolved against the project's ``models/`` directory.
        """
        try:
            with open(filepath, "r") as fp:
                raw = yaml.safe_load(fp) or {}
        except OSError as e:
            raise AegisConfigError(f"Failed to read environment config {filepath!r}: {e}") from e
        except yaml.YAMLError as e:
            raise AegisConfigError(f"Failed to parse environment config {filepath!r}: {e}") from e

        if not isinstance(raw, dict):
            raise AegisConfigError(
                f"Environment config {filepath!r} must be a mapping at the top level; got {type(raw).__name__}"
            )
        return cls.from_yaml_dict(raw)

    @classmethod
    def from_yaml_dict(cls, raw: dict) -> Self:
        """
        Build an EnvironmentConfig from an already-parsed YAML mapping.
        """
        assert_keys_match_attrs(cls, raw, "environment_config")

        extra_models_raw = raw.get("extra_models") or []
        if not isinstance(extra_models_raw, list):
            raise AegisConfigError(f"'extra_models' must be a list; got {type(extra_models_raw).__name__}")
        extra_models: list[StaticModelConfig] = []
        for idx, item in enumerate(extra_models_raw):
            context = f"extra_models[{idx}]"
            if not isinstance(item, dict):
                raise AegisConfigError(f"{context} must be a mapping; got {type(item).__name__}")
            extra_models.append(StaticModelConfig.from_yaml_dict(item, context))

        return cls(
            manipulator_base_xyz=_parse_xyz(raw.get("manipulator_base_xyz"), "manipulator_base_xyz"),
            manipulator_base_rpy=_parse_xyz(raw.get("manipulator_base_rpy"), "manipulator_base_rpy"),
            extra_models=tuple(extra_models),
        )
