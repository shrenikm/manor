"""
Round-trip tests for RGBImageData.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.tests.factories import random_rgb_image_data


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_rgb_image_data(rng)
    assert RGBImageData.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_rgb_image_data(rng)
    assert RGBImageData.from_lcm_message(original.to_lcm_message()) == original
