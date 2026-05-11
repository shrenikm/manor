"""
Hatchling custom build hook that compiles LCM schemas and validates Cap'n Proto schemas during
uv pip install --no-cache-dir -e .
"""

from __future__ import annotations

import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface

_REPO_ROOT = Path(__file__).resolve().parent.parent


class CustomBuildHook(BuildHookInterface):
    PLUGIN_NAME = "custom"

    def initialize(self, version: str, build_data: dict) -> None:
        scripts_dir = _REPO_ROOT / "scripts"
        if str(scripts_dir) not in sys.path:
            sys.path.insert(0, str(scripts_dir))
        import compile_messages  # type: ignore[import-not-found]

        compile_messages.main()
