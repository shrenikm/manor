"""
Tests for the IManipulatorVariant abstract base and its registry.
"""

from __future__ import annotations

import pytest

from manor.common.exceptions import UnknownManipulatorTypeError, VariantAlreadyRegisteredError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import (
    IManipulatorVariant,
    get_registered_manipulator_types,
    get_variant_class,
    register_manipulator_variant,
)


class TestVariantRegistry:
    def test_lite6_variant_class_is_registered(self) -> None:
        assert get_variant_class(ManipulatorType.LITE6) is Lite6Variant

    def test_unknown_manipulator_type_raises(self) -> None:
        # Construct a fresh enum member that isn't in the registry by
        # going through StrEnum's ``_missing_`` path with a bogus value.
        with pytest.raises(UnknownManipulatorTypeError):
            get_variant_class("__unregistered__")  # type: ignore[arg-type]

    def test_lite6_listed_in_registered_types(self) -> None:
        assert ManipulatorType.LITE6 in get_registered_manipulator_types()


class TestVariantAlreadyRegistered:
    def test_double_registration_raises(self) -> None:
        # Re-registering Lite6Variant under the same manipulator type
        # must fail rather than silently overwriting.
        with pytest.raises(VariantAlreadyRegisteredError):

            @register_manipulator_variant(ManipulatorType.LITE6)
            class _DummyDuplicate(IManipulatorVariant):
                X = "x"

                def get_manipulator_type(self) -> ManipulatorType:
                    return ManipulatorType.LITE6


if __name__ == "__main__":
    run_manor_tests()
