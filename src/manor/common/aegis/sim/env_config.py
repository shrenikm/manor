"""
Environment configuration for the aegis simulator.

An ``EnvironmentConfig`` describes how the manipulator is mounted and
which static models populate the world around it. Configurations can be
authored as YAML and loaded through ``EnvironmentConfig.from_yaml``; the
default config (no extras, manipulator welded to the world origin) is
also available via ``EnvironmentConfig.default()``.

YAML schema (all fields optional):

```yaml
manipulator_base_xyz: [0.0, 0.0, 0.0]
manipulator_base_rpy: [0.0, 0.0, 0.0]
extra_models:
  - name: table
    description_filepath: /abs/path/to/table.urdf
    base_xyz: [0.0, 0.0, 0.0]
    base_rpy: [0.0, 0.0, 0.0]
    weld_to_world: true
```
"""

from __future__ import annotations

from typing import Self

import attr
import numpy as np
import yaml

from manor.common.custom_types import FilePath, NpVector3f64
from manor.common.exceptions import EnvironmentConfigError


def _zero_xyz() -> NpVector3f64:
    return np.zeros(3, dtype=np.float64)


def _zero_rpy() -> NpVector3f64:
    return np.zeros(3, dtype=np.float64)


@attr.frozen
class StaticModelConfig:
    """
    A single static model loaded into the simulation world.

    The model is loaded from ``description_filepath``. If
    ``weld_to_world`` is true, the Sim explicitly welds the model's
    base body to the world frame at the configured pose.

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
        """
        try:
            with open(filepath, "r") as fp:
                raw = yaml.safe_load(fp) or {}
        except OSError as e:
            raise EnvironmentConfigError(f"Failed to read environment config {filepath!r}: {e}") from e
        except yaml.YAMLError as e:
            raise EnvironmentConfigError(f"Failed to parse environment config {filepath!r}: {e}") from e

        if not isinstance(raw, dict):
            raise EnvironmentConfigError(
                f"Environment config {filepath!r} must be a mapping at the top level; got {type(raw).__name__}"
            )

        manipulator_base_xyz = _parse_xyz(raw.get("manipulator_base_xyz"), "manipulator_base_xyz")
        manipulator_base_rpy = _parse_xyz(raw.get("manipulator_base_rpy"), "manipulator_base_rpy")

        extra_models_raw = raw.get("extra_models") or []
        if not isinstance(extra_models_raw, list):
            raise EnvironmentConfigError(f"'extra_models' must be a list; got {type(extra_models_raw).__name__}")
        extra_models = tuple(_parse_static_model(item, idx) for idx, item in enumerate(extra_models_raw))

        return cls(
            manipulator_base_xyz=manipulator_base_xyz,
            manipulator_base_rpy=manipulator_base_rpy,
            extra_models=extra_models,
        )


def _parse_xyz(value: object, field_name: str) -> NpVector3f64:
    if value is None:
        return np.zeros(3, dtype=np.float64)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise EnvironmentConfigError(f"'{field_name}' must be a length-3 list of floats; got {value!r}")
    try:
        return np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as e:
        raise EnvironmentConfigError(f"'{field_name}' could not be coerced to a float vector: {e}") from e


def _parse_static_model(raw: object, idx: int) -> StaticModelConfig:
    if not isinstance(raw, dict):
        raise EnvironmentConfigError(f"extra_models[{idx}] must be a mapping; got {type(raw).__name__}")
    name = raw.get("name")
    description_filepath = raw.get("description_filepath")
    if not isinstance(name, str) or not name:
        raise EnvironmentConfigError(f"extra_models[{idx}].name is required and must be a non-empty string")
    if not isinstance(description_filepath, str) or not description_filepath:
        raise EnvironmentConfigError(
            f"extra_models[{idx}].description_filepath is required and must be a non-empty string"
        )
    weld_to_world = raw.get("weld_to_world", True)
    if not isinstance(weld_to_world, bool):
        raise EnvironmentConfigError(
            f"extra_models[{idx}].weld_to_world must be a bool; got {type(weld_to_world).__name__}"
        )
    return StaticModelConfig(
        name=name,
        description_filepath=description_filepath,
        base_xyz=_parse_xyz(raw.get("base_xyz"), f"extra_models[{idx}].base_xyz"),
        base_rpy=_parse_xyz(raw.get("base_rpy"), f"extra_models[{idx}].base_rpy"),
        weld_to_world=weld_to_world,
    )
