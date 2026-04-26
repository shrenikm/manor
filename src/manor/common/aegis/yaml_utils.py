"""
YAML parsing helpers shared across aegis config classes.

Every aegis config drives its own YAML key set and per-field parsing
off its attrs definition; ``parse_attrs_yaml`` is the canonical way
to do that. It walks ``attr.fields(cls)``, reads each field's
``.type`` directly, and dispatches parsing off that type:

* ``X | None`` -- ``null`` stays ``None``; otherwise recurse with X.
* ``enum.Enum`` subclass (incl. ``StrEnum``) -- enum constructor.
* class with a classmethod ``from_yaml_dict`` -- recurse through it.
* ``bool`` / ``int`` / ``float`` / ``str`` -- matching ``require_*``.
* PEP 695 ``type`` aliases (``TypeAliasType``) -- unwrap to the
  underlying type and recurse.

Anything outside that set (numpy arrays, ``IManipulatorModel``,
``DrakeLcm``, ``tuple[T, ...]`` collections) is handled via the
``custom_parsers`` / ``skip_fields`` escape hatches the helper exposes.

The primitive validators (``require_int`` / ``require_number`` /
``require_bool`` / ``require_str`` / ``require_dict``) keep value-type
checking uniform across configs so error messages and edge cases
(notably ``bool`` being a subclass of ``int``) are handled in one
place.
"""

from __future__ import annotations

import enum
import types
from typing import Callable, TypeAliasType, Union, get_args, get_origin

import attr

from manor.common.exceptions import AegisConfigError


def require_int(value: object, field_name: str) -> int:
    """
    Coerce a YAML scalar to ``int``. Rejects ``bool`` explicitly
    because ``bool`` is a subclass of ``int`` in Python and would
    otherwise sneak through without a type error.
    """
    if isinstance(value, bool) or not isinstance(value, int):
        raise AegisConfigError(f"'{field_name}' must be an int; got {type(value).__name__}")
    return value


def require_number(value: object, field_name: str) -> float:
    """
    Coerce a YAML scalar to ``float``. Accepts both ``int`` and
    ``float`` source values; rejects ``bool`` for the same reason as
    ``require_int``.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AegisConfigError(f"'{field_name}' must be a number; got {type(value).__name__}")
    return float(value)


def require_bool(value: object, field_name: str) -> bool:
    """
    Coerce a YAML scalar to ``bool`` -- strict on type so a stray
    ``0`` / ``1`` doesn't get accepted as a boolean flag.
    """
    if not isinstance(value, bool):
        raise AegisConfigError(f"'{field_name}' must be a bool; got {type(value).__name__}")
    return value


def require_str(value: object, field_name: str, *, allow_empty: bool = False) -> str:
    """
    Coerce a YAML scalar to ``str``. ``allow_empty`` lets callers opt
    into accepting empty strings (e.g. an unset hardware ``serial_number``).
    """
    if not isinstance(value, str):
        raise AegisConfigError(f"'{field_name}' must be a string; got {type(value).__name__}")
    if not allow_empty and not value:
        raise AegisConfigError(f"'{field_name}' must be a non-empty string")
    return value


def require_dict(value: object, field_name: str) -> dict:
    """
    Coerce a YAML scalar to ``dict``; rejects everything else with a
    helpful error pointing at ``field_name``.
    """
    if not isinstance(value, dict):
        raise AegisConfigError(f"'{field_name}' must be a mapping; got {type(value).__name__}")
    return value


def parse_attrs_yaml(
    cls: type,
    raw: dict,
    context: str,
    *,
    custom_parsers: dict[str, Callable[[object, str], object]] | None = None,
    skip_fields: set[str] | None = None,
) -> dict[str, object]:
    """
    Walk the attrs fields of ``cls`` and parse the matching YAML keys.
    Returns a kwargs dict ready to splat into ``cls(**...)``.

    YAML keys must match attrs field names; unknown keys raise. ClassVar
    attributes (e.g. ``POLICY_TYPE``, ``SYSTEM_NAME``) are excluded
    automatically because attrs doesn't enumerate them as instance
    fields, which is exactly what we want here.

    Required-vs-optional follows attrs: a field with no default is
    required (missing keys raise ``AegisConfigError``); fields with
    defaults / factories are simply omitted from the result when absent
    so the attrs default takes effect.
    """
    custom_parsers = custom_parsers or {}
    skip_fields = skip_fields or set()

    # ``from __future__ import annotations`` makes attr.field.type a
    # string. resolve_types() rewrites those strings to actual classes
    # so the dispatch in _parse_field_value can use isinstance/issubclass
    # directly. Idempotent; safe to call repeatedly.
    try:
        attr.resolve_types(cls)
    except (NameError, AttributeError):
        # Forward refs that aren't importable from the class's module
        # will still surface as a clear error inside _parse_field_value.
        pass

    fields = attr.fields(cls)
    allowed = {field.name for field in fields if field.name not in skip_fields}
    extras = set(raw) - allowed
    if extras:
        raise AegisConfigError(f"{context}: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}")

    parsed: dict[str, object] = {}
    for field in fields:
        if field.name in skip_fields:
            continue
        field_context = f"{context}.{field.name}"
        if field.name not in raw:
            if field.default is attr.NOTHING:
                raise AegisConfigError(f"{field_context} is required")
            continue
        value = raw[field.name]
        if field.name in custom_parsers:
            parsed[field.name] = custom_parsers[field.name](value, field_context)
            continue
        parsed[field.name] = _parse_field_value(field.type, value, field_context)
    return parsed


def _parse_field_value(field_type: object, value: object, context: str) -> object:
    """
    Dispatch parsing of ``value`` off ``field_type`` (an actual class
    or generic alias). Recurses on ``X | None`` and on PEP 695 type
    aliases.
    """
    # PEP 695 ``type Foo = ...`` aliases: unwrap to the underlying type.
    if isinstance(field_type, TypeAliasType):
        return _parse_field_value(field_type.__value__, value, context)

    origin = get_origin(field_type)

    # ``X | None`` / ``Optional[X]``: pass through explicit null,
    # otherwise recurse with the non-None branch.
    if origin is Union or origin is types.UnionType:
        args = get_args(field_type)
        if value is None and type(None) in args:
            return None
        non_none = [arg for arg in args if arg is not type(None)]
        if len(non_none) == 1:
            return _parse_field_value(non_none[0], value, context)
        raise AegisConfigError(f"{context}: cannot auto-parse union type {field_type!r}; pass a custom_parsers entry")

    if isinstance(field_type, type):
        # ``bool`` is a subclass of ``int``, so it must be checked first.
        if issubclass(field_type, bool):
            return require_bool(value, context)
        if issubclass(field_type, enum.Enum):
            try:
                return field_type(value)
            except ValueError as e:
                raise AegisConfigError(
                    f"Unknown {context}: {value!r}; expected one of {[m.value for m in field_type]}"
                ) from e
        if issubclass(field_type, int):
            return require_int(value, context)
        if issubclass(field_type, float):
            return require_number(value, context)
        if issubclass(field_type, str):
            return require_str(value, context)
        # Nested attrs config (or any class that knows how to parse
        # itself from a YAML mapping). The base class for tagged-union
        # configs (MetisPolicyConfigBase / KyberControllerConfigBase)
        # also implements this and dispatches through its manager,
        # which is what lets the helper recurse into them by field type
        # alone.
        if hasattr(field_type, "from_yaml_dict"):
            return field_type.from_yaml_dict(require_dict(value, context))

    raise AegisConfigError(
        f"{context}: no parser registered for type {field_type!r}; "
        "pass a custom_parsers entry or list the field in skip_fields"
    )
