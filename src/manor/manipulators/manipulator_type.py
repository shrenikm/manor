"""
Top-level manipulator-type enum.

One member per supported manipulator family. Variant enums (per-trim configurations like vacuum vs parallel
gripper) live alongside each manipulator's package and reference the matching ManipulatorType.
"""

from __future__ import annotations

from enum import StrEnum


class ManipulatorType(StrEnum):
    """
    Identifies a manipulator family (independent of trim / variant).
    """

    LITE6 = "lite6"
