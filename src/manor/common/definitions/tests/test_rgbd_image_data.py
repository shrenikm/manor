"""
Round-trip tests for RGBDImageData.
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.rgbd_image_data import RGBDImageData
from manor.common.definitions.tests.factories import random_rgbd_image_data
from manor.common.testing_utils import run_manor_tests


def test_capnp_roundtrip(rng: np.random.Generator) -> None:
    original = random_rgbd_image_data(rng)
    assert RGBDImageData.deserialize(original.serialize()) == original


def test_lcm_roundtrip(rng: np.random.Generator) -> None:
    original = random_rgbd_image_data(rng)
    assert RGBDImageData.from_lcm_message(original.to_lcm_message()) == original


if __name__ == "__main__":
    run_manor_tests()
