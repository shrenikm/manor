#!/usr/bin/env bash
# Launch lcm-spy with the manor LCM type bindings on the Java classpath, so
# messages on AEGIS_* channels (and any other manor channel) show up decoded
# rather than as raw bytes.
#
# The required jar is produced by scripts/compile_messages.py, which runs
# automatically on `uv pip install --no-cache-dir -e .`. If it's missing,
# either reinstall the package or run the compile script directly.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
MANOR_JAR="$REPO_ROOT/build/java/manor_lcmtypes.jar"

if [[ ! -f "$MANOR_JAR" ]]; then
  echo "manor_lcmtypes.jar not found at:" >&2
  echo "    $MANOR_JAR" >&2
  echo "Build it with one of:" >&2
  echo "    uv pip install --no-cache-dir -e .   # rebuild via the hatch build hook" >&2
  echo "    python $SCRIPT_DIR/compile_messages.py" >&2
  exit 1
fi

if ! command -v lcm-spy >/dev/null 2>&1; then
  echo "lcm-spy not found on PATH. Activate the manor conda environment first." >&2
  exit 1
fi

# lcm-spy's wrapper script appends $CLASSPATH to its internal classpath, so we
# just have to export our jar here.
exec env CLASSPATH="$MANOR_JAR${CLASSPATH:+:$CLASSPATH}" lcm-spy "$@"
