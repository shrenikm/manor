"""
Round-trip tests for DepthImageData.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.tests.factories import random_depth_image_data


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_depth_image_data(rng)
    assert DepthImageData.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_depth_image_data(rng)
    assert DepthImageData.from_lcm_message(original.to_lcm_message()) == original
