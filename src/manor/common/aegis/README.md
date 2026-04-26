# aegis

The aegis stack runs the manor manipulation pipeline as N independent
OS processes that talk to each other over LCM. Same diagram shape in
sim and on hardware; only the per-block backends and the process layout
differ.

## Blocks

| block      | mode(s)        | role                                                                                       |
| ---------- | -------------- | ------------------------------------------------------------------------------------------ |
| **metis**  | sim + hardware | runs the policy. Subscribes to proprioception + RGB + depth, publishes action.             |
| **gylos**  | sim only       | the simulated world: gaia + advancer + helios (sim) + talos (sim) + kyber, all in one process. Subscribes to action, publishes proprioception + RGB + depth. |
| **kylos**  | hardware only  | kyber + talos against the real arm driver. Subscribes to action, publishes proprioception. |
| **helios** | hardware only  | publishes RGB + depth from the real camera SDK.                                            |

Sim mode runs `metis` + `gylos` (2 processes). Hardware mode runs
`metis` + `kylos` + `helios` (3 processes).

Why the asymmetry? `Helios`'s sim backend closes over a Python handle
to `Gaia`; that closure can't cross a process boundary, so in sim the
helios role is fulfilled inside the gylos process. On hardware
`Helios` is its own process because its backend talks to a camera SDK,
not Gaia. Same logic for kyber+talos: in sim they live inside gylos
because talos needs the in-process Gaia handle; on hardware they live
in kylos.

LCM channels are defined in `aegis_utils.AegisChannel`.

## Quick start

```bash
# In the manor conda env
aegis run gylos       # spawn the sim world (Meshcat URL prints to stdout)
aegis run metis       # spawn the policy
aegis status          # confirm both alive
aegis kill metis
aegis kill gylos
```

That's it for the typical sim session. Open the printed Meshcat URL in a
browser to watch the plant.

## CLI reference

```text
aegis [-c CONFIG] <command>
```

Flags:

- `-c PATH`, `--config PATH` — aegis YAML to load. Defaults to
  `configs/aegis/lite6_default.yaml` (resolved from the manor repo
  root). The YAML is parsed and validated once; bad configs fail fast.
- `-h`, `--help` — works at the top level and on every subcommand.

Commands:

| command                | what it does                                                          |
| ---------------------- | --------------------------------------------------------------------- |
| `run <block>`          | spawn `<block>` as a subprocess; refuses if it's already running or doesn't apply to the configured mode. |
| `kill <block>`         | SIGTERM `<block>`'s subprocess.                                       |
| `status [<block>]`     | report `<block>`'s state (alive + PID, or stopped). With no arg, equivalent to `list`. |
| `list`                 | list every block applicable to the current mode + state.              |
| `repl`                 | drop into an interactive prompt_toolkit shell.                        |

State persists across shells via `/tmp/aegis_<block>.pid`. Stale PID
files (process gone, file remained) are auto-cleaned on the next
`status`/`run`.

## REPL

```bash
aegis repl
```

Drops you into:

```text
aegis repl -- mode=sim, config=/.../lite6_default.yaml
type 'help' for commands, 'exit' or Ctrl-D to leave (children keep running).
aegis>
```

Inside, the same commands minus the `aegis` prefix:

```text
aegis> list
aegis> run gylos
aegis> run metis
aegis> status
aegis> kill metis
aegis> kill gylos
aegis> exit
```

- Tab-complete: `run g<TAB>` → `run gylos`.
- Arrow-up recalls past lines (history persists at `~/.aegis_history`).
- Ctrl-C clears the current line; Ctrl-D / `exit` / `quit` leaves the
  REPL. Spawned children keep running.

## Standalone runners

Each per-block runner is also runnable directly as `python -m`. Useful
for dev iteration when you don't want to go through the supervisor.

```bash
# Convert the YAML to JSON once
python -c "import json, yaml; print(json.dumps(yaml.safe_load(open('configs/aegis/lite6_default.yaml'))))" > /tmp/aegis.json

# In one terminal
python -m manor.common.aegis.run.run_gylos < /tmp/aegis.json

# In another terminal
python -m manor.common.aegis.run.run_metis < /tmp/aegis.json
```

The runner reads the full parsed-AegisConfig dict from stdin and slices
out only the keys it needs. Ctrl-C in either terminal stops that block;
no PID files are written.

## Observing LCM traffic

In a third terminal:

```bash
timeout 5 lcm-logger /tmp/aegis_capture.log
python -c "
import lcm
log = lcm.EventLog('/tmp/aegis_capture.log')
counts = {}
for ev in log:
    counts[ev.channel] = counts.get(ev.channel, 0) + 1
for ch, n in sorted(counts.items()):
    print(f'{ch}: {n}')
"
```

You should see:

- `AEGIS_ACTION` — metis publishing.
- `AEGIS_PROPRIOCEPTION` — talos publishing (via gylos in sim, kylos on hw).
- `AEGIS_RGB_IMAGE` / `AEGIS_DEPTH_IMAGE` — helios publishing (via gylos in sim, helios process on hw).

Counts are typically below the configured frequencies because gylos
runs slightly below realtime (lots of periodic events plus continuous
integration); the wiring is correct as long as every channel fires at
least once.

## Configuration

The default config lives at `configs/aegis/lite6_default.yaml`. Top-level
keys map 1:1 to attrs fields on `AegisConfig`; sub-blocks recurse the
same way. Every block is **required** in the YAML — the schema is the
explicit source of truth, no implicit factory defaults at the top
level. Inner fields within a block can omit defaults (e.g.
`environment_config: {}` is a valid block).

To switch to a different policy or controller, edit
`metis_config.policy_config.type` (one of `MetisPolicyType`) or
`kyber_config.controller_config.type` (one of `KyberControllerType`).
To turn the visualiser on/off, flip `gaia_config.enable_meshcat`. To
change sim playback speed, edit `gaia_config.target_realtime_rate`
(`0.0` = as fast as possible, `1.0` = real time).

The CLI accepts `--config` to load a different YAML; per-field overrides
are deferred until there's a real need.

## Tests

```bash
python -m pytest src/manor/common/aegis -q
```

Runs all aegis unit + smoke tests (diagram-build, YAML round-trip,
runner imports, CLI commands). The CLI tests sandbox PID files into a
`tmp_path` so they don't collide with a running aegis stack.

## Caveats / known limits

- **The arm doesn't move on command yet.** `Gaia.apply_joint_*_command`
  stashes the latest command but the plant's actuation port is force-
  fixed to zero. The pipeline ticks correctly end-to-end; the visible
  behaviour is the arm sitting (or drooping under gravity, depending on
  the URDF). Wiring a tracking controller into Gaia is the obvious next
  milestone.
- **Talos's EEF FK is stubbed.** `_compute_eef_pose` /
  `_compute_eef_twist` return identity / zeros. Doesn't affect the
  pipeline; just means downstream consumers of EEF state get
  placeholders.
- **gylos runs below realtime.** kyber at 500 Hz + helios at 30 Hz +
  gaia advancer at 500 Hz + continuous-time integration adds up.
  Switching `gaia_config.time_step` to a small discrete value (e.g.
  `0.001`) speeds it up at the cost of integration accuracy.
