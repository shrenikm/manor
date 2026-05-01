"""
Reusable attrs field validators for definitions and other attrs classes.

Each function returns a validator callable suitable for the ``validator=`` kwarg
on ``attr.field(...)``. Validators here focus on a single attribute (shape,
dtype, value-range constraints). Cross-attribute invariants belong in the
class's ``__attrs_post_init__``.

All validators raise InvalidDefinitionError on failure so that constructor
failures share a single exception type with the cross-attribute checks in
__attrs_post_init__.
"""

from __future__ import annotations

from typing import Any, Callable

import attr
import numpy as np

from manor.common.exceptions import InvalidDefinitionError


_QUATERNION_NORM_TOLERANCE: float = 1e-6


Validator = Callable[[Any, attr.Attribute, Any], None]


def _qualified_name(instance: Any, attribute: attr.Attribute) -> str:
    return f"{type(instance).__name__}.{attribute.name}"


def is_ndarray(*, dtype: np.dtype | type | None = np.float64) -> Validator:
    """
    Require the attribute to be an np.ndarray (and optionally pin its dtype).
    """

    def _validate(instance: Any, attribute: attr.Attribute, value: Any) -> None:
        if not isinstance(value, np.ndarray):
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must be an ndarray; got {type(value).__name__}"
            )
        if dtype is not None and value.dtype != np.dtype(dtype):
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must have dtype {np.dtype(dtype)}; got {value.dtype}"
            )

    return _validate


def is_1d_array(*, dtype: np.dtype | type | None = np.float64, min_size: int = 0) -> Validator:
    """
    Require a 1-D ndarray of the given dtype, optionally with a minimum length.
    """
    base = is_ndarray(dtype=dtype)

    def _validate(instance: Any, attribute: attr.Attribute, value: np.ndarray) -> None:
        base(instance, attribute, value)
        if value.ndim != 1:
            raise InvalidDefinitionError(f"{_qualified_name(instance, attribute)} must be 1-D; got shape {value.shape}")
        if value.shape[0] < min_size:
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must have at least {min_size} elements; got {value.shape[0]}"
            )

    return _validate


def is_2d_array(
    *,
    dtype: np.dtype | type | None = np.float64,
    min_rows: int = 0,
    min_cols: int = 0,
    expected_cols: int | None = None,
) -> Validator:
    """
    Require a 2-D ndarray of the given dtype, optionally with minimum row /
    column counts or an exact column count.
    """
    base = is_ndarray(dtype=dtype)

    def _validate(instance: Any, attribute: attr.Attribute, value: np.ndarray) -> None:
        base(instance, attribute, value)
        if value.ndim != 2:
            raise InvalidDefinitionError(f"{_qualified_name(instance, attribute)} must be 2-D; got shape {value.shape}")
        if value.shape[0] < min_rows:
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must have at least {min_rows} rows; got {value.shape[0]}"
            )
        if value.shape[1] < min_cols:
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must have at least {min_cols} columns; got {value.shape[1]}"
            )
        if expected_cols is not None and value.shape[1] != expected_cols:
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must have exactly {expected_cols} columns; got {value.shape[1]}"
            )

    return _validate


def has_shape(shape: tuple[int, ...], *, dtype: np.dtype | type | None = np.float64) -> Validator:
    """
    Require an ndarray of an exact shape.
    """
    base = is_ndarray(dtype=dtype)

    def _validate(instance: Any, attribute: attr.Attribute, value: np.ndarray) -> None:
        base(instance, attribute, value)
        if value.shape != shape:
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must have shape {shape}; got {value.shape}"
            )

    return _validate


def is_unit_quaternion(*, atol: float = _QUATERNION_NORM_TOLERANCE) -> Validator:
    """
    Require a length-4 ndarray with unit norm (within atol).
    """
    base = has_shape((4,), dtype=np.float64)

    def _validate(instance: Any, attribute: attr.Attribute, value: np.ndarray) -> None:
        base(instance, attribute, value)
        norm = float(np.linalg.norm(value))
        if not np.isclose(norm, 1.0, atol=atol):
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must be a unit quaternion (norm 1, tol {atol}); got norm {norm}"
            )

    return _validate


def is_non_decreasing_1d() -> Validator:
    """
    Require a 1-D ndarray whose entries are non-decreasing (allows ties).
    """
    base = is_1d_array(dtype=np.float64)

    def _validate(instance: Any, attribute: attr.Attribute, value: np.ndarray) -> None:
        base(instance, attribute, value)
        if value.shape[0] >= 2 and not np.all(np.diff(value) >= 0.0):
            raise InvalidDefinitionError(f"{_qualified_name(instance, attribute)} must be non-decreasing")

    return _validate


def is_non_negative_int() -> Validator:
    """
    Require a non-negative Python int. Rejects bool to avoid accidental True/False.
    """

    def _validate(instance: Any, attribute: attr.Attribute, value: Any) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must be an int; got {type(value).__name__}"
            )
        if value < 0:
            raise InvalidDefinitionError(f"{_qualified_name(instance, attribute)} must be >= 0; got {value}")

    return _validate


def is_non_negative_number() -> Validator:
    """
    Require a non-negative real number (int or float, not bool, not NaN).
    """

    def _validate(instance: Any, attribute: attr.Attribute, value: Any) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise InvalidDefinitionError(
                f"{_qualified_name(instance, attribute)} must be a real number; got {type(value).__name__}"
            )
        if isinstance(value, float) and not np.isfinite(value):
            raise InvalidDefinitionError(f"{_qualified_name(instance, attribute)} must be finite; got {value}")
        if value < 0:
            raise InvalidDefinitionError(f"{_qualified_name(instance, attribute)} must be >= 0; got {value}")

    return _validate
