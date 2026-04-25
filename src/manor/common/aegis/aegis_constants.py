"""
Shared constants for the Aegis sub-systems.

All LCM channel names, adapter port names, and adapter-internal
system-role tags live here so no aegis code has to embed those strings
as raw literals.
"""

from __future__ import annotations

from enum import StrEnum


class AegisChannel(StrEnum):
    """
    Canonical LCM channel names for every aegis message type.

    All aegis pub/sub wiring (sub-system tests, run scripts, the future
    consolidated builder) should reference these members rather than raw
    channel-name strings.
    """

    PROPRIOCEPTION = "AEGIS_PROPRIOCEPTION"
    ACTION = "AEGIS_ACTION"
    COMMAND = "AEGIS_COMMAND"
    RGB_IMAGE = "AEGIS_RGB_IMAGE"
    DEPTH_IMAGE = "AEGIS_DEPTH_IMAGE"
    OBSERVATION = "AEGIS_OBSERVATION"


class AegisAdapterPorts(StrEnum):
    """
    Shared port-name enum for all aegis adapters.

    Subscriber adapters expose ``DEFINITION_OUTPUT``; publisher adapters
    expose ``DEFINITION_INPUT``. The translator LeafSystems inside the
    LCM adapter pair use the LCM_* members for their LCM-typed sides.
    Future non-LCM adapters can add members here rather than introducing
    parallel enums.
    """

    LCM_INPUT = "lcm_input"
    LCM_OUTPUT = "lcm_output"
    DEFINITION_INPUT = "definition_input"
    DEFINITION_OUTPUT = "definition_output"


class AegisAdapterSystemRole(StrEnum):
    """
    Role tags used as system names for the leaves inside an adapter
    diagram, so ``set_name`` calls don't embed raw strings.
    """

    SUBSCRIBER = "subscriber"
    PUBLISHER = "publisher"
    LCM_TO_DEFINITION_TRANSLATOR = "lcm_to_definition_translator"
    DEFINITION_TO_LCM_TRANSLATOR = "definition_to_lcm_translator"
