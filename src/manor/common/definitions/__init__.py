"""
Typed data-class definitions for state, action, observation, and commands.

Each definition is a frozen attrs class with Cap'n Proto serialization and
an LCM message counterpart. LCM python classes are emitted by the build hook
into the sibling `_generated/` directory; we expose that directory on sys.path
so the generated `manor_lcm` package is importable as a top-level module (LCM
cross-references other types in the same package via `import manor_lcm`, which
requires `manor_lcm` to resolve as a top-level import).
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path

_GENERATED_DIR = str(_Path(__file__).parent / "_generated")
if _GENERATED_DIR not in _sys.path:
    _sys.path.insert(0, _GENERATED_DIR)
