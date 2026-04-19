"""
Capability interfaces and shared base class for typed data-class definitions.

Every definition is a frozen attrs class that subclasses DefinitionBase and
therefore supports Cap'n Proto serialization (ISerializable) and LCM pub/sub
conversion (ILcmMessage).
"""

from __future__ import annotations

import abc
from typing import Any, ClassVar, Self

from manor.common.exceptions import SerializationError


class ISerializable(abc.ABC):
    """
    Capability for round-tripping a value to/from bytes.
    """

    @abc.abstractmethod
    def serialize(self) -> bytes: ...

    @classmethod
    @abc.abstractmethod
    def deserialize(cls, data: bytes) -> Self: ...


class ILcmMessage(abc.ABC):
    """
    Capability for converting a value to/from a generated LCM message class.
    """

    @abc.abstractmethod
    def to_lcm_message(self) -> Any: ...

    @classmethod
    @abc.abstractmethod
    def from_lcm_message(cls, msg: Any) -> Self: ...


class DefinitionBase(ISerializable, ILcmMessage):
    """
    Base for frozen attrs data-class definitions that carry Cap'n Proto versioning
    and an LCM message counterpart.

    Subclasses MUST set:
        VERSION                    : semver string, e.g. "1.0.0".
        CAPNP_SCHEMA               : the loaded Cap'n Proto VersionedFoo struct type.
        LCM_CLASS                  : the generated LCM type for pub/sub.
        CURRENT_CAPNP_UNION_ARM    : the union arm to write on serialize ("v1", "v2", ...).

    Subclasses MUST implement:
        _to_capnp_current(self, builder) -> None
            Fill the current-version struct builder.
        _from_capnp_v{N}(cls, reader) -> Self
            One classmethod per supported capnp version. Migrations from older
            versions to the latest python shape live here.
        to_lcm_message(self) -> LCM_CLASS
        from_lcm_message(cls, msg) -> Self
    """

    VERSION: ClassVar[str]
    CAPNP_SCHEMA: ClassVar[Any]
    LCM_CLASS: ClassVar[type]
    CURRENT_CAPNP_UNION_ARM: ClassVar[str] = "v1"

    def serialize(self) -> bytes:
        try:
            msg = self.CAPNP_SCHEMA.new_message()
            inner = msg.init(self.CURRENT_CAPNP_UNION_ARM)
            self._to_capnp_current(inner)
            return msg.to_bytes()
        except SerializationError:
            raise
        except Exception as e:
            raise SerializationError(f"Failed to serialize {type(self).__name__}: {e}") from e

    @classmethod
    def deserialize(cls, data: bytes) -> Self:
        try:
            with cls.CAPNP_SCHEMA.from_bytes(data) as msg:
                arm = msg.which()
                if arm == "unset":
                    raise SerializationError(
                        f"{cls.__name__} bytes have the sentinel 'unset' arm active "
                        "-- payload was never populated with a version"
                    )
                handler = getattr(cls, f"_from_capnp_{arm}", None)
                if handler is None:
                    raise SerializationError(
                        f"{cls.__name__} has no handler for capnp union arm {arm!r}"
                    )
                return handler(getattr(msg, arm))
        except SerializationError:
            raise
        except Exception as e:
            raise SerializationError(f"Failed to deserialize {cls.__name__}: {e}") from e

    # Hook methods required on subclasses. Not declared abstract to keep attrs.frozen
    # subclass instantiation clean; ABC enforcement is handled by to_lcm_message /
    # from_lcm_message inherited from ILcmMessage.

    def _to_capnp_current(self, builder: Any) -> None:
        raise NotImplementedError

    @classmethod
    def _from_capnp_v1(cls, reader: Any) -> Self:
        raise NotImplementedError
