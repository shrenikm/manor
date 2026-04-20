"""
Capability interfaces and shared base class for typed data-class definitions.

Every definition is a frozen attrs class that subclasses DefinitionBase and
therefore supports Cap'n Proto serialization (ISerializable) and LCM pub/sub
conversion (ILcmMessage).
"""

from __future__ import annotations

import abc
from typing import Any, ClassVar, Self, override

from manor.common.definitions.utils.capnp_utils import CapnpStructSchema, CapnpUnionArm
from manor.common.exceptions import SerializationError


class ISerializable(abc.ABC):
    """
    Capability for round-tripping a value to/from bytes via Cap'n Proto.
    """

    @abc.abstractmethod
    def serialize(self) -> bytes: ...

    @classmethod
    @abc.abstractmethod
    def deserialize(cls, data: bytes) -> Self: ...

    @classmethod
    @abc.abstractmethod
    def get_capnp_schema(cls) -> CapnpStructSchema: ...


class ILcmMessage(abc.ABC):
    """
    Capability for converting a value to/from a generated LCM message class.
    """

    @abc.abstractmethod
    def to_lcm_message(self) -> Any: ...

    @classmethod
    @abc.abstractmethod
    def from_lcm_message(cls, msg: Any) -> Self: ...

    @classmethod
    @abc.abstractmethod
    def get_lcm_class(cls) -> type: ...


class DefinitionBase(ISerializable, ILcmMessage):
    """
    Base for frozen attrs data-class definitions that carry Cap'n Proto versioning
    and an LCM message counterpart.

    Subclasses MUST set:
        CURRENT_CAPNP_VERSION      : the union arm to write on serialize ("v1", "v2", ...).

    Subclasses MUST implement:
        get_capnp_schema(cls) -> CapnpStructSchema
            Return the VersionedX wrapper struct type for this definition.
        get_lcm_class(cls) -> type
            Return the generated LCM class used for pub/sub.
        _to_capnp_current(self, builder) -> None
            Fill the current-version struct builder.
        _from_capnp_v{N}(cls, reader) -> Self
            One classmethod per supported capnp version. Migrations from older
            versions to the latest python shape live here.
        to_lcm_message(self) -> <LCM class>
        from_lcm_message(cls, msg) -> Self

    Composite definitions call `_to_versioned_capnp` / `_from_versioned_capnp`
    on their nested definitions. These helpers operate on a VersionedX
    builder/reader (i.e. the union wrapper), so nested fields can be declared
    in the parent schema as `VersionedX` rather than a specific `XV1` and each
    definition can evolve its version independently.
    """

    CURRENT_CAPNP_VERSION: ClassVar[str] = "v1"

    def _to_versioned_capnp(self, versioned_builder: Any) -> None:
        """
        Init the current-version arm of a VersionedX builder and fill it.
        """
        inner = versioned_builder.init(self.CURRENT_CAPNP_VERSION)
        self._to_capnp_current(inner)

    @classmethod
    def _from_versioned_capnp(cls, versioned_reader: Any) -> Self:
        """
        Read a VersionedX reader, dispatching on the active arm to the
        matching `_from_capnp_v{N}` handler.
        """
        arm = versioned_reader.which()
        if arm == CapnpUnionArm.UNSET:
            raise SerializationError(
                f"{cls.__name__} versioned reader has sentinel '{CapnpUnionArm.UNSET}' arm "
                "active -- payload was never populated with a version"
            )
        handler = getattr(cls, f"_from_capnp_{arm}", None)
        if handler is None:
            raise SerializationError(f"{cls.__name__} has no handler for capnp union arm {arm!r}")
        return handler(getattr(versioned_reader, arm))

    @override
    def serialize(self) -> bytes:
        try:
            schema = self.get_capnp_schema()
            msg = schema.new_message()
            self._to_versioned_capnp(msg)
            return msg.to_bytes()
        except SerializationError:
            raise
        except Exception as e:
            raise SerializationError(f"Failed to serialize {type(self).__name__}: {e}") from e

    @classmethod
    @override
    def deserialize(cls, data: bytes) -> Self:
        try:
            schema = cls.get_capnp_schema()
            with schema.from_bytes(data) as msg:
                return cls._from_versioned_capnp(msg)
        except SerializationError:
            raise
        except Exception as e:
            raise SerializationError(f"Failed to deserialize {cls.__name__}: {e}") from e
