"""
Per-block runners for the aegis stack.

Each module under this package builds the slice of the aegis Drake
diagram that lives inside one process and runs it forever (until
SIGTERM / SIGINT). The CLI / REPL spawns these as subprocesses --
they are also runnable standalone for dev iteration:

    # compose the base + sub-yamls into an inlined JSON payload
    python -c "import json; from manor.common.aegis.aegis import compose_aegis_yaml_dict; print(json.dumps(compose_aegis_yaml_dict('configs/aegis/lite6_ac.yaml')))" > /tmp/aegis.json
    # then feed it in over stdin
    python -m manor.common.aegis.run.run_metis < /tmp/aegis.json

Each per-block module exposes a ``run_<block>(...)`` function that
takes already-parsed config(s) and a ``DrakeLcm`` handle, plus a
``__main__`` entry that parses a JSON config payload from stdin.
"""
