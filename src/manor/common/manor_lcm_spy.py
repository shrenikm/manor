"""
Console-script entry point for ``lcm-spy`` with the manor LCM type bindings
on the Java classpath.

The required jar (``build/java/manor_lcmtypes.jar``) is produced by
``scripts/compile_messages.py``, which runs automatically on
``uv pip install --no-cache-dir -e .``. If it's missing, either reinstall
the package or run the compile script directly.

This module exists so the ``manor_lcm_spy`` command can be exposed via
``[project.scripts]`` in ``pyproject.toml``; it relies on an editable
install so ``__file__`` resolves under the manor repo and the jar can be
located by walking up to the repo root.
"""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

_LCM_SPY_BIN = "lcm-spy"
_MANOR_JAR_RELATIVE = Path("build") / "java" / "manor_lcmtypes.jar"


def _find_repo_root() -> Path:
    """
    Walk up from this file until a directory containing ``pyproject.toml``
    is found. Editable installs leave ``__file__`` pointing at the source
    tree, so this resolves to the manor repo root.
    """
    here = Path(__file__).resolve()
    for candidate in (here, *here.parents):
        if (candidate / "pyproject.toml").is_file():
            return candidate
    raise RuntimeError(
        "Could not locate the manor repo root from "
        f"{here!s}. manor_lcm_spy requires an editable install (uv pip install --no-cache-dir -e .)."
    )


def main() -> int:
    repo_root = _find_repo_root()
    manor_jar = repo_root / _MANOR_JAR_RELATIVE
    if not manor_jar.is_file():
        print(f"manor_lcmtypes.jar not found at:\n    {manor_jar}", file=sys.stderr)
        print("Build it with one of:", file=sys.stderr)
        print("    uv pip install --no-cache-dir -e .   # rebuild via the hatch build hook", file=sys.stderr)
        print(f"    python {repo_root / 'scripts' / 'compile_messages.py'}", file=sys.stderr)
        return 1

    if shutil.which(_LCM_SPY_BIN) is None:
        print(f"{_LCM_SPY_BIN} not found on PATH. Activate the manor conda environment first.", file=sys.stderr)
        return 1

    # lcm-spy's wrapper script appends $CLASSPATH to its internal classpath,
    # so we just have to export our jar before exec'ing.
    existing_classpath = os.environ.get("CLASSPATH", "")
    new_classpath = f"{manor_jar}{os.pathsep}{existing_classpath}" if existing_classpath else str(manor_jar)
    env = {**os.environ, "CLASSPATH": new_classpath}
    os.execvpe(_LCM_SPY_BIN, [_LCM_SPY_BIN, *sys.argv[1:]], env)


if __name__ == "__main__":
    sys.exit(main())
