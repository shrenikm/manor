"""
Shared utilities for the Aegis sub-systems.

LCM channel names and adapter-internal system-role tags live here so no aegis code has to embed those strings as raw
literals. Adapter port enums are defined alongside the adapter class itself in aegis_adapters.py (matching the
per-system convention used by Kyber / Helios / Talos / Metis).
"""

from __future__ import annotations

from enum import StrEnum


class AegisChannel(StrEnum):
    """
    Canonical LCM channel names for every aegis message type.

    All aegis pub/sub wiring (sub-system tests, run scripts, the future consolidated builder) should reference these
    members rather than raw channel-name strings.
    """

    PROPRIOCEPTION = "AEGIS_PROPRIOCEPTION"
    ACTION = "AEGIS_ACTION"
    JOINT_EE_COMMAND = "AEGIS_JOINT_EE_COMMAND"
    RGB_IMAGE = "AEGIS_RGB_IMAGE"
    DEPTH_IMAGE = "AEGIS_DEPTH_IMAGE"
    OBSERVATION = "AEGIS_OBSERVATION"


class AegisAdapterSystemRole(StrEnum):
    """
    Role tags used as system names for the leaves inside an adapter diagram, so set_name calls don't embed raw strings.
    """

    SUBSCRIBER = "subscriber"
    PUBLISHER = "publisher"
    LCM_TO_DEFINITION_TRANSLATOR = "lcm_to_definition_translator"
    DEFINITION_TO_LCM_TRANSLATOR = "definition_to_lcm_translator"
