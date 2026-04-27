# xarm-python-sdk: Lite6 API notes

Empirical notes captured while bringing up the Ufactory Lite6 against
[`xarm-python-sdk`](https://github.com/xArm-Developer/xArm-Python-SDK)
(installed as the `xarm` package). Each section corresponds to one
experiment in `scripts/lite6_standalone.py`. The goal is to record
quirks, return codes, and timing observations that aren't obvious
from the SDK source — so we can lean on this file when wiring the
hardware backends inside aegis.

When something here is `TODO`, it means the experiment is written but
hasn't been run on metal yet. When it's a fact, it's been verified.

## Setup

- Default Lite6 IP: **192.168.1.178** (configurable per-instance,
  override via `--ip` on every script).
- `XArmAPI(port=<ip>, is_radian=True)` — pass `is_radian=True` once at
  construction so every angle method defaults to radians for the rest
  of the session.
- `from xarm.wrapper import XArmAPI` prints `SDK_VERSION: <ver>` to
  stdout on import with no off switch — redirect stdout while loading
  to keep our own output clean (`scripts/lite6_standalone.py` does this).

## Activation (`prime` / `unprime`)

Sequence used by both `Lite6Driver.prime()` and `lite6_standalone.prime`:

1. `clean_error()` — wipe any latched fault before enabling motors.
2. `motion_enable(enable=True)` — turn motors on (audible relay click).
3. `set_mode(mode=1)` — servo position mode (lets `set_servo_angle_j`
   work in a streaming loop).
4. `set_state(state=0)` — `READY`; required before motion calls.

Inverse for shutdown:

1. `set_state(state=4)` — `STOP`.
2. `motion_enable(enable=False)`.
3. `disconnect()` — close the TCP session.

### xarm mode constants

| value | name             | use                                                   |
| ----- | ---------------- | ----------------------------------------------------- |
| `0`   | position         | motion-plan-style API (`set_position`, `set_servo_angle`) |
| `1`   | servo position   | low-latency joint streaming via `set_servo_angle_j`   |
| `4`   | joint velocity   | `vc_set_joint_velocity`                                |

(There are other modes — these are the three manor uses.)

### xarm state constants

| value | name    |
| ----- | ------- |
| `0`   | READY   |
| `4`   | STOP    |

## Reading joint state

`get_joint_states(is_radian=True)` returns `(code, (positions, velocities, torques))`.

- All three vectors are 7-element regardless of arm DOF; the Lite6 has
  6 joints, so slice `[:6]`. The 7th slot is reserved for 7-DOF arms.
- `code == 0` is the success sentinel; any other code we should treat
  as a hard failure (raise / abort).
- TODO: confirm units — assumed rad / rad·s⁻¹ when `is_radian=True`,
  but worth printing a sanity-check at known poses (joint 1 at +π/2,
  rest of arm slack) and comparing to the teach pendant readout.

## Quirks to investigate

These will get filled in as we run the matching experiment.

- TODO: behaviour of `get_joint_states` when the e-stop is engaged —
  does it return the last cached state, error out, or block?
- TODO: does `clean_error()` succeed silently when no error is latched,
  or does it return a non-zero code?
- TODO: how long does `motion_enable(True)` take to return? (matters
  for `prime()` startup latency and aegis boot ordering)
- TODO: what happens if `set_servo_angle_j` is called without
  `set_mode(1)` first — error code or accepted-but-ignored?
- TODO: rate ceiling for `set_servo_angle_j` streaming (manor wants
  500 Hz; the SDK doc claims 250 Hz).
- TODO: is there any `get_lite6_gripper_*` call that reports gripper
  position / state? (driver currently returns `None` for both EEF
  reads on the assumption that there isn't one.)
- TODO: `vc_set_joint_velocity` semantics — does `duration=0` mean
  "until next command", or "one tick"? does sending zeros stop, or
  drift?

## Return-code reference

Captured codes seen in the wild (will populate as we hit them):

| code | seen during              | what it meant                                        |
| ---- | ------------------------ | ---------------------------------------------------- |
| `0`  | normal operation         | success                                              |

(SDK source has the full list under `xarm/core/code.py` if we need to
look up an unfamiliar code mid-experiment.)
