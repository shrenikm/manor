"""
Abstract base for per-manipulator variant enums plus the cross-manipulator registries that map a
``ManipulatorType`` to its concrete variant enum class and its concrete ``IManipulatorModel``
class.

Each manipulator package defines a concrete variant subclass enumerating its trims and registers
it via ``register_manipulator_variant``; it also registers its model class via
``register_manipulator_model``. The registries let cross-manipulator code (CLIs, config loaders,
the aegis YAML parser) construct a model for a ``(type, variant)`` pair without hardcoding the
manipulator family in every dispatch site.
"""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Callable
from enum import StrEnum
from typing import TYPE_CHECKING, TypeVar

from manor.common.exceptions import UnknownManipulatorTypeError, VariantAlreadyRegisteredError
from manor.manipulators.manipulator_type import ManipulatorType

if TYPE_CHECKING:
    from manor.manipulators.manipulator_model import IManipulatorModel


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
_MODEL_REGISTRY: dict[ManipulatorType, type["IManipulatorModel"]] = {}

# Bound TypeVar so the decorator preserves the concrete class type
# (rather than collapsing it to ``type[IManipulatorVariant]``, which
# would hide member access from type checkers like pyright).
_VariantT = TypeVar("_VariantT", bound=IManipulatorVariant)
_ModelT = TypeVar("_ModelT", bound="IManipulatorModel")


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


def register_manipulator_model(
    manipulator_type: ManipulatorType,
) -> Callable[[type[_ModelT]], type[_ModelT]]:
    """
    Class decorator that registers a concrete ``IManipulatorModel`` subclass for a given
    ``ManipulatorType``. The decorated class must accept a single ``variant`` keyword argument
    of the corresponding variant enum type. Used by ``build_manipulator_model`` so any
    cross-manipulator code can construct a model from a ``(type, variant)`` pair without
    hardcoding the family-to-model dispatch in every callsite.
    """

    def _register(cls: type[_ModelT]) -> type[_ModelT]:
        if manipulator_type in _MODEL_REGISTRY:
            raise VariantAlreadyRegisteredError(
                f"Model class already registered for {manipulator_type!r}: {_MODEL_REGISTRY[manipulator_type].__name__}"
            )
        _MODEL_REGISTRY[manipulator_type] = cls
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


def get_model_class(manipulator_type: ManipulatorType) -> type["IManipulatorModel"]:
    """
    Return the concrete ``IManipulatorModel`` subclass registered for ``manipulator_type``.
    Raises ``UnknownManipulatorTypeError`` if no model class is registered.
    """

    if manipulator_type not in _MODEL_REGISTRY:
        raise UnknownManipulatorTypeError(
            f"No model class registered for {manipulator_type!r}. Ensure the matching manipulator package is imported."
        )
    return _MODEL_REGISTRY[manipulator_type]


def build_manipulator_model(
    manipulator_type: ManipulatorType,
    variant: IManipulatorVariant,
) -> "IManipulatorModel":
    """
    Construct an ``IManipulatorModel`` for a ``(type, variant)`` pair via the registered model
    class. The variant must match the registered variant class for the given type; otherwise
    a TypeError will surface from the model's constructor.
    """
    return get_model_class(manipulator_type)(variant=variant)


def get_registered_manipulator_types() -> tuple[ManipulatorType, ...]:
    """
    Return all ``ManipulatorType``s that currently have a registered
    variant class.
    """

    return tuple(_VARIANT_REGISTRY.keys())
