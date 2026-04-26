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

When javac / jar / lcm.jar are all present, Java bindings for the same LCM
schemas are also compiled and bundled into ``build/java/manor_lcmtypes.jar``
for use with ``lcm-spy`` (exposed via the ``manor_lcm_spy`` console
script). This step is optional -- it is soft-skipped with a warning when
any of the tools is missing, so headless / install-only environments
still succeed.
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

_JAVA_BUILD_DIR = _REPO_ROOT / "build" / "java"
_JAVA_SOURCES_DIR = _JAVA_BUILD_DIR / "sources"
_JAVA_CLASSES_DIR = _JAVA_BUILD_DIR / "classes"
_MANOR_LCMTYPES_JAR = _JAVA_BUILD_DIR / "manor_lcmtypes.jar"


def _compile_lcm_schemas() -> None:
    """
    Wipe lcmtypes/ and regenerate it by running lcm-gen on every .lcm file.

    Each .lcm file declares `package manor.common.definitions.lcmtypes;` so
    lcm-gen with --ppath src/ drops its output directly into the package tree.
    """
    lcm_gen = shutil.which("lcm-gen")
    if lcm_gen is None:
        raise RuntimeError(
            "lcm-gen not found on PATH. Ensure the 'lcm' package is installed in the active environment."
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


def _find_lcm_jar() -> Path | None:
    """
    Locate the lcm.jar that ships alongside the installed ``lcm`` Python
    package, falling back to a couple of standard system paths.
    """
    try:
        import lcm as _lcm

        site_packages_java = Path(_lcm.__file__).resolve().parent.parent / "share" / "java" / "lcm.jar"
    except ImportError:
        site_packages_java = None

    candidates: list[Path | None] = [
        site_packages_java,
        Path("/usr/share/java/lcm.jar"),
        Path("/usr/local/share/java/lcm.jar"),
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate
    return None


def _compile_lcm_java_bindings() -> None:
    """
    Build ``manor_lcmtypes.jar`` from the same LCM schemas used for Python.

    Soft-skips (prints a warning and returns) if javac, jar, lcm-gen, or
    lcm.jar are missing -- the jar is only needed to decode Aegis messages
    inside lcm-spy, so a missing Java toolchain must not break the rest of
    the install.
    """
    javac = shutil.which("javac")
    jar_tool = shutil.which("jar")
    lcm_gen = shutil.which("lcm-gen")
    if not (javac and jar_tool and lcm_gen):
        missing = [name for name, tool in [("javac", javac), ("jar", jar_tool), ("lcm-gen", lcm_gen)] if tool is None]
        print(
            f"[compile_messages] skipping Java LCM bindings: missing {', '.join(missing)}. "
            "Install a JDK to enable message decoding in lcm-spy.",
            file=sys.stderr,
        )
        return

    lcm_jar = _find_lcm_jar()
    if lcm_jar is None:
        print(
            "[compile_messages] skipping Java LCM bindings: could not locate lcm.jar. "
            "Re-install the 'lcm' package or extend _find_lcm_jar() to point at your install.",
            file=sys.stderr,
        )
        return

    lcm_files = sorted(_LCM_SCHEMA_DIR.glob("*.lcm"))
    if not lcm_files:
        return

    if _JAVA_BUILD_DIR.exists():
        shutil.rmtree(_JAVA_BUILD_DIR)
    _JAVA_SOURCES_DIR.mkdir(parents=True)
    _JAVA_CLASSES_DIR.mkdir(parents=True)

    subprocess.run(
        [lcm_gen, "--java", "--jpath", str(_JAVA_SOURCES_DIR), *[str(p) for p in lcm_files]],
        check=True,
    )

    java_sources = sorted(_JAVA_SOURCES_DIR.rglob("*.java"))
    if not java_sources:
        print("[compile_messages] lcm-gen produced no Java sources; skipping javac / jar.", file=sys.stderr)
        return

    subprocess.run(
        [javac, "-cp", str(lcm_jar), "-d", str(_JAVA_CLASSES_DIR), *[str(p) for p in java_sources]],
        check=True,
    )

    subprocess.run(
        [jar_tool, "cf", str(_MANOR_LCMTYPES_JAR), "-C", str(_JAVA_CLASSES_DIR), "."],
        check=True,
    )
    print(f"[compile_messages] built {_MANOR_LCMTYPES_JAR} ({len(lcm_files)} types)")


def main() -> int:
    _compile_lcm_schemas()
    _validate_capnp_schemas()
    _compile_lcm_java_bindings()
    return 0


if __name__ == "__main__":
    sys.exit(main())
