"""
Shared pytest fixtures for definition round-trip tests.
"""

from __future__ import annotations

import numpy as np
import pytest

_SEED = 42


@pytest.fixture
def rng() -> np.random.Generator:
    """
    Deterministic RNG for test instance generation.
    """
    return np.random.default_rng(_SEED)
