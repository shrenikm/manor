"""
Compile LCM schemas into Python classes and validate Cap'n Proto schemas.

Invoked by the hatchling build hook on `uv pip install --no-cache-dir -e .`
and can also be run directly:

    python scripts/compile_messages.py

LCM python modules are emitted in-place under the package tree so they're
importable as `manor.common.definitions.lcmtypes.<name>`. The entire lcmtypes/
directory is wiped and regenerated on each run.

Cap'n Proto schemas are validated by loading each top-level wrapper schema
with a fresh SchemaParser.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC_ROOT = _REPO_ROOT / "src"
_DEFINITIONS_DIR = _SRC_ROOT / "manor" / "common" / "definitions"
_LCM_SCHEMA_DIR = _DEFINITIONS_DIR / "schemas" / "lcm"
_CAPNP_SCHEMA_DIR = _DEFINITIONS_DIR / "schemas" / "capnp"
_LCMTYPES_DIR = _DEFINITIONS_DIR / "lcmtypes"


def _compile_lcm_schemas() -> None:
    """
    Wipe lcmtypes/ and regenerate it by running lcm-gen on every .lcm file.

    Each .lcm file declares `package manor.common.definitions.lcmtypes;` so
    lcm-gen with --ppath src/ drops its output directly into the package tree.
    """
    lcm_gen = shutil.which("lcm-gen")
    if lcm_gen is None:
        raise RuntimeError(
            "lcm-gen not found on PATH. Ensure the 'lcm' package is installed "
            "in the active environment."
        )

    if _LCMTYPES_DIR.exists():
        shutil.rmtree(_LCMTYPES_DIR)

    lcm_files = sorted(_LCM_SCHEMA_DIR.glob("*.lcm"))
    if not lcm_files:
        return

    cmd = [
        lcm_gen,
        "--python",
        "--ppath",
        str(_SRC_ROOT),
        *[str(p) for p in lcm_files],
    ]
    subprocess.run(cmd, check=True)


def _validate_capnp_schemas() -> None:
    """
    Load every .capnp schema with a fresh parser to fail fast on schema errors.
    """
    import capnp

    schemas = sorted(_CAPNP_SCHEMA_DIR.glob("*.capnp"))
    for path in schemas:
        parser = capnp.SchemaParser()
        parser.load(str(path), imports=[str(_CAPNP_SCHEMA_DIR)])


def main() -> int:
    _compile_lcm_schemas()
    _validate_capnp_schemas()
    return 0


if __name__ == "__main__":
    sys.exit(main())
