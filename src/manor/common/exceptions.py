class ManorError(Exception):
    """
    Base exception for the Manor project.
    """

    pass


class Lite6Error(ManorError):
    pass


class ManipulatorError(ManorError):
    """
    Base exception for the manipulators sub-package.
    """

    pass


class UnknownManipulatorTypeError(ManipulatorError):
    """
    Raised when a ManipulatorType has no registered variant class or
    when a lookup against the registry fails for some other reason.
    """

    pass


class VariantAlreadyRegisteredError(ManipulatorError):
    """
    Raised when register_variant_for is called for a manipulator type that already has a variant
    class registered.
    """

    pass


class ManipulatorDriverError(ManipulatorError):
    """
    Base exception for hardware-driver failures.
    """

    pass


class Lite6DriverError(ManipulatorDriverError):
    """
    Raised when a Lite6Driver call (prime/unprime/read/write) fails.
    """

    pass


class AegisError(ManorError):
    """
    Base exception for the aegis sub-system.
    """

    pass


class AegisConfigError(AegisError):
    """
    Raised when any aegis configuration (top-level, per-subsystem,
    or environment YAML) cannot be parsed or fails validation.
    """

    pass


class GaiaError(AegisError):
    """
    Raised when the Gaia simulator encounters a runtime error.
    """

    pass


class DefinitionError(ManorError):
    """
    Base exception for errors related to typed data-class definitions.
    """

    pass


class InvalidDefinitionError(DefinitionError):
    """
    Raised when a definition's invariants (e.g. exactly-one-of fields) are violated.
    """

    pass


class SerializationError(DefinitionError):
    """
    Raised when serializing or deserializing a definition fails.
    """

    pass
