"""
Abstract base for per-manipulator variant enums plus a registry mapping
``ManipulatorType`` to its concrete variant class.

Each manipulator package defines a concrete subclass enumerating its
trims and registers it via the ``register_manipulator_variant`` decorator.
The registry lets cross-manipulator code (CLIs, config loaders) discover
"all variants for this manipulator" without hardcoding the class name.
"""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Callable
from enum import StrEnum
from typing import TypeVar

from manor.common.exceptions import UnknownManipulatorTypeError, VariantAlreadyRegisteredError
from manor.manipulators.manipulator_type import ManipulatorType


class IManipulatorVariant(StrEnum):
    """
    Abstract base for manipulator-specific variant enums.

    Concrete subclasses (e.g. ``Lite6Variant``) enumerate the trims
    available for a given manipulator and implement
    ``get_manipulator_type`` to identify the family they belong to.
    """

    @abstractmethod
    def get_manipulator_type(self) -> ManipulatorType: ...


_VARIANT_REGISTRY: dict[ManipulatorType, type[IManipulatorVariant]] = {}

# Bound TypeVar so the decorator preserves the concrete class type
# (rather than collapsing it to ``type[IManipulatorVariant]``, which
# would hide member access from type checkers like pyright).
_VariantT = TypeVar("_VariantT", bound=IManipulatorVariant)


def register_manipulator_variant(
    manipulator_type: ManipulatorType,
) -> Callable[[type[_VariantT]], type[_VariantT]]:
    """
    Class decorator that registers a variant enum class for a given
    ``ManipulatorType``.
    """

    def _register(cls: type[_VariantT]) -> type[_VariantT]:
        if manipulator_type in _VARIANT_REGISTRY:
            raise VariantAlreadyRegisteredError(
                f"Variant class already registered for {manipulator_type!r}: "
                f"{_VARIANT_REGISTRY[manipulator_type].__name__}"
            )
        _VARIANT_REGISTRY[manipulator_type] = cls
        return cls

    return _register


def get_variant_class(manipulator_type: ManipulatorType) -> type[IManipulatorVariant]:
    """
    Return the concrete ``IManipulatorVariant`` subclass registered for
    ``manipulator_type``. Raises ``UnknownManipulatorTypeError`` if no
    variant class is registered (typically because the corresponding
    manipulator package was never imported).
    """

    if manipulator_type not in _VARIANT_REGISTRY:
        raise UnknownManipulatorTypeError(
            f"No variant class registered for {manipulator_type!r}. "
            "Ensure the matching manipulator package is imported."
        )
    return _VARIANT_REGISTRY[manipulator_type]


def get_registered_manipulator_types() -> tuple[ManipulatorType, ...]:
    """
    Return all ``ManipulatorType``s that currently have a registered
    variant class.
    """

    return tuple(_VARIANT_REGISTRY.keys())
