from enum import StrEnum


class ManipulatorControlTarget(StrEnum):
    """
    Enum to represent the control target of a manipulator.
    """

    JOINT = "joint"
    EEF = "eef"


class ManipulatorControlType(StrEnum):
    # Position control. Can be for any control target.
    POSITION = "position"
    # Velocity control. Can be for any control target.
    VELOCITY = "velocity"
    # Control both position and velocity.
    STATE = "state"


class ManipulatorGripperStatus(StrEnum):
    """
    Enum to represent the status of a manipulator's gripper.
    Assumes parallel gripper for now.
    """

    OPENING = "opening"
    CLOSING = "closing"
    NEUTRAL = "neutral"
    OPEN = "open"
    CLOSED = "closed"
