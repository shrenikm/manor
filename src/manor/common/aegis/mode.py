"""
Aegis execution modes.

Selects the backend implementation for systems that touch the outside world
(Helios sensors, Talos manipulator). Systems without a sim/hardware split
(Metis, Kyber) are mode-agnostic and behave identically in both modes.
"""

from enum import StrEnum


class AegisMode(StrEnum):
    SIM = "sim"
    HARDWARE = "hardware"
