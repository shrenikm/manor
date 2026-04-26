"""
Per-block runners for the aegis stack.

Each module under this package builds the slice of the aegis Drake
diagram that lives inside one process and runs it forever (until
SIGTERM / SIGINT). The CLI / REPL spawns these as subprocesses --
they are also runnable standalone for dev iteration:

    python -m manor.common.aegis.run.metis < <(jq .metis_config configs/aegis/lite6_default.yaml)

Each per-block module exposes a ``run_<block>(...)`` function that
takes already-parsed config(s) and a ``DrakeLcm`` handle, plus a
``__main__`` entry that parses a JSON config payload from stdin.
"""
