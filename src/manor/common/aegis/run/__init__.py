"""
Per-block runners for the aegis stack.

Each module under this package builds the slice of the aegis Drake
diagram that lives inside one process and runs it forever (until
SIGTERM / SIGINT). The CLI / REPL spawns these as subprocesses --
they are also runnable standalone for dev iteration:

    # convert the YAML to JSON once
    python -c "import json, yaml; print(json.dumps(yaml.safe_load(open('configs/aegis/default_ac.yaml'))))" > /tmp/aegis.json
    # then feed it in over stdin
    python -m manor.common.aegis.run.run_metis < /tmp/aegis.json

Each per-block module exposes a ``run_<block>(...)`` function that
takes already-parsed config(s) and a ``DrakeLcm`` handle, plus a
``__main__`` entry that parses a JSON config payload from stdin.
"""
