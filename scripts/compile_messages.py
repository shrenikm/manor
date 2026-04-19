"""
Compile LCM schemas into Python classes and validate Cap'n Proto schemas.

Invoked by the hatchling build hook on `uv pip install --no-cache-dir -e .`
and can also be run directly:

    python scripts/compile_messages.py

Emits generated LCM Python modules to:
    src/manor/common/definitions/_generated/

and verifies that every .capnp schema in
    src/manor/common/definitions/schemas/capnp/
loads without error.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_DEFINITIONS_DIR = _REPO_ROOT / "src" / "manor" / "common" / "definitions"
_LCM_SCHEMA_DIR = _DEFINITIONS_DIR / "schemas" / "lcm"
_CAPNP_SCHEMA_DIR = _DEFINITIONS_DIR / "schemas" / "capnp"
_GENERATED_DIR = _DEFINITIONS_DIR / "_generated"


def _compile_lcm_schemas() -> None:
    """
    Run lcm-gen on every .lcm file and place outputs under _generated/.
    """
    lcm_gen = shutil.which("lcm-gen")
    if lcm_gen is None:
        raise RuntimeError(
            "lcm-gen not found on PATH. Ensure the 'lcm' package is installed "
            "in the active environment."
        )

    _GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    (_GENERATED_DIR / "__init__.py").touch()

    lcm_files = sorted(_LCM_SCHEMA_DIR.glob("*.lcm"))
    if not lcm_files:
        return

    cmd = [
        lcm_gen,
        "--python",
        "--ppath",
        str(_GENERATED_DIR),
        *[str(p) for p in lcm_files],
    ]
    subprocess.run(cmd, check=True)


def _validate_capnp_schemas() -> None:
    """
    Load every top-level wrapper .capnp schema with a fresh parser to fail fast
    on schema errors. Per-version and shared files load transitively.
    """
    import capnp

    wrapper_schemas = []
    for path in sorted(_CAPNP_SCHEMA_DIR.rglob("*.capnp")):
        name = path.stem
        # Skip per-version schemas (loaded transitively by their wrapper).
        if "_v" in name and name.rsplit("_v", 1)[-1].isdigit():
            continue
        wrapper_schemas.append(path)

    for path in wrapper_schemas:
        parser = capnp.SchemaParser()
        parser.load(str(path), imports=[str(_CAPNP_SCHEMA_DIR)])


def main() -> int:
    _compile_lcm_schemas()
    _validate_capnp_schemas()
    return 0


if __name__ == "__main__":
    sys.exit(main())
