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
