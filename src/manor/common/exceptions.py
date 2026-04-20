class ManorError(Exception):
    """
    Base exception for the Manor project.
    """

    pass


class Lite6Error(ManorError):
    pass


class Lite6PliantError(Lite6Error):
    pass


class Lite6SimulationPliantError(Lite6PliantError):
    pass


class Lite6HardwarePliantError(Lite6PliantError):
    pass


class Lite6PliantChoreographerError(Lite6PliantError):
    pass


class Lite6SystemError(Lite6Error):
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
