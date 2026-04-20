"""
Project-wide pytest fixtures.

Anything placed here is available to every test in the manor package. Keep it
narrowly scoped to things that make sense across the whole codebase.
"""

from __future__ import annotations

import numpy as np
import pytest

_RNG_SEED = 42


@pytest.fixture
def rng() -> np.random.Generator:
    """
    Deterministic numpy RNG seeded for reproducibility.

    Use in tests that need random inputs: request this fixture by adding
    `rng: np.random.Generator` to the test signature and call methods like
    `rng.uniform(...)`, `rng.standard_normal(...)`, `rng.integers(...)`,
    `rng.choice(...)`, `rng.bytes(...)`.
    """
    return np.random.default_rng(_RNG_SEED)
