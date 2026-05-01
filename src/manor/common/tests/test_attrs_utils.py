"""
Tests for the reusable attrs validators in common/attrs_utils.py.
"""

from __future__ import annotations

import attr
import numpy as np
import pytest

from manor.common.attrs_utils import (
    has_shape,
    is_1d_array,
    is_2d_array,
    is_ndarray,
    is_non_decreasing_1d,
    is_non_negative_int,
    is_non_negative_number,
    is_unit_quaternion,
)
from manor.common.exceptions import InvalidDefinitionError
from manor.common.testing_utils import run_manor_tests


def _make_class(validator):
    @attr.frozen
    class _Holder:
        value: object = attr.field(validator=validator)

    return _Holder


class TestIsNdarray:
    def test_accepts_ndarray(self) -> None:
        cls = _make_class(is_ndarray())
        cls(value=np.zeros(3, dtype=np.float64))

    def test_rejects_non_ndarray(self) -> None:
        cls = _make_class(is_ndarray())
        with pytest.raises(InvalidDefinitionError):
            cls(value=[1.0, 2.0])

    def test_rejects_wrong_dtype(self) -> None:
        cls = _make_class(is_ndarray(dtype=np.float64))
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros(3, dtype=np.int32))

    def test_dtype_none_accepts_any_dtype(self) -> None:
        cls = _make_class(is_ndarray(dtype=None))
        cls(value=np.zeros(3, dtype=np.int32))


class TestIs1dArray:
    def test_accepts_1d(self) -> None:
        cls = _make_class(is_1d_array())
        cls(value=np.zeros(0, dtype=np.float64))
        cls(value=np.zeros(5, dtype=np.float64))

    def test_rejects_2d(self) -> None:
        cls = _make_class(is_1d_array())
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros((2, 2), dtype=np.float64))

    def test_min_size_enforced(self) -> None:
        cls = _make_class(is_1d_array(min_size=3))
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros(2, dtype=np.float64))
        cls(value=np.zeros(3, dtype=np.float64))


class TestIs2dArray:
    def test_accepts_2d(self) -> None:
        cls = _make_class(is_2d_array())
        cls(value=np.zeros((3, 4), dtype=np.float64))

    def test_rejects_1d(self) -> None:
        cls = _make_class(is_2d_array())
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros(3, dtype=np.float64))

    def test_min_rows_cols_enforced(self) -> None:
        cls = _make_class(is_2d_array(min_rows=2, min_cols=2))
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros((1, 2), dtype=np.float64))
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros((2, 1), dtype=np.float64))
        cls(value=np.zeros((2, 2), dtype=np.float64))

    def test_expected_cols_enforced(self) -> None:
        cls = _make_class(is_2d_array(expected_cols=4))
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros((3, 5), dtype=np.float64))
        cls(value=np.zeros((3, 4), dtype=np.float64))


class TestHasShape:
    def test_accepts_matching_shape(self) -> None:
        cls = _make_class(has_shape((3,)))
        cls(value=np.zeros(3, dtype=np.float64))

    def test_rejects_mismatched_shape(self) -> None:
        cls = _make_class(has_shape((3,)))
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.zeros(4, dtype=np.float64))


class TestIsUnitQuaternion:
    def test_accepts_unit_quaternion(self) -> None:
        cls = _make_class(is_unit_quaternion())
        cls(value=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))
        q = np.array([0.1, -0.2, 0.3, 0.5], dtype=np.float64)
        cls(value=q / np.linalg.norm(q))

    def test_rejects_wrong_shape(self) -> None:
        cls = _make_class(is_unit_quaternion())
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.array([1.0, 0.0, 0.0], dtype=np.float64))

    def test_rejects_non_unit_norm(self) -> None:
        cls = _make_class(is_unit_quaternion())
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.array([2.0, 0.0, 0.0, 0.0], dtype=np.float64))


class TestIsNonDecreasing1d:
    def test_accepts_non_decreasing(self) -> None:
        cls = _make_class(is_non_decreasing_1d())
        cls(value=np.array([], dtype=np.float64))
        cls(value=np.array([0.0], dtype=np.float64))
        cls(value=np.array([0.0, 0.0, 1.5, 1.5, 3.0], dtype=np.float64))

    def test_rejects_decreasing(self) -> None:
        cls = _make_class(is_non_decreasing_1d())
        with pytest.raises(InvalidDefinitionError):
            cls(value=np.array([0.0, 1.0, 0.5], dtype=np.float64))


class TestIsNonNegativeInt:
    def test_accepts_zero_and_positive(self) -> None:
        cls = _make_class(is_non_negative_int())
        cls(value=0)
        cls(value=12345)

    def test_rejects_negative(self) -> None:
        cls = _make_class(is_non_negative_int())
        with pytest.raises(InvalidDefinitionError):
            cls(value=-1)

    def test_rejects_float_and_bool(self) -> None:
        cls = _make_class(is_non_negative_int())
        with pytest.raises(InvalidDefinitionError):
            cls(value=1.0)
        with pytest.raises(InvalidDefinitionError):
            cls(value=True)


class TestIsNonNegativeNumber:
    def test_accepts_zero_positive_float_and_int(self) -> None:
        cls = _make_class(is_non_negative_number())
        cls(value=0)
        cls(value=0.0)
        cls(value=3.14)

    def test_rejects_negative(self) -> None:
        cls = _make_class(is_non_negative_number())
        with pytest.raises(InvalidDefinitionError):
            cls(value=-0.001)

    def test_rejects_nan_and_inf(self) -> None:
        cls = _make_class(is_non_negative_number())
        with pytest.raises(InvalidDefinitionError):
            cls(value=float("nan"))
        with pytest.raises(InvalidDefinitionError):
            cls(value=float("inf"))


if __name__ == "__main__":
    run_manor_tests()
