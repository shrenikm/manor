"""
Round-trip tests for TimestampHeader.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.tests.factories import random_timestamp_header
from manor.common.definitions.timestamp_header import TimestampHeader


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_timestamp_header(rng)
    assert TimestampHeader.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_timestamp_header(rng)
    assert TimestampHeader.from_lcm_message(original.to_lcm_message()) == original
