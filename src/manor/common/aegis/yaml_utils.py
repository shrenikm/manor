"""
YAML parsing helpers shared across aegis config classes.

Every aegis config drives its own YAML key set off ``attr.fields``;
``assert_keys_match_attrs`` is the canonical "no unknown keys" check
that every ``from_yaml_dict`` runs first. The primitive validators
(``require_int`` / ``require_number`` / ``require_bool`` /
``require_str``) keep value-type checking uniform across configs so
error messages and edge cases (notably ``bool`` being a subclass of
``int``) are handled in one place.
"""

from __future__ import annotations

import attr

from manor.common.exceptions import AegisConfigError


def assert_keys_match_attrs(cls: type, d: dict, context: str) -> None:
    """
    Raise ``AegisConfigError`` when ``d`` contains keys outside the
    attrs fields of ``cls``. ClassVars (e.g. ``POLICY_TYPE`` /
    ``CONTROLLER_TYPE``) are intentionally excluded because attrs
    treats them as class-level constants, not instance fields, which
    is exactly what we want here -- they're the discriminator the
    manager dispatches on, not a yaml-side knob.
    """
    allowed = {field.name for field in attr.fields(cls)}
    extras = set(d) - allowed
    if extras:
        raise AegisConfigError(f"{context}: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}")


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
