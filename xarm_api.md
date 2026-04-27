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

### Observed: motors wiggle and settle on `motion_enable(True)`

When the CLI prints `priming...` and runs `motion_enable(True)`, the
arm visibly micro-moves for a couple of seconds before settling. Not
a homing routine (Lite6 uses absolute encoders); it's the controller
releasing the brakes, reading each encoder, and locking the servo
loop's setpoint onto the current measured position. Happens every
time we re-enable after a disconnect.

`prime()` now sleeps `_MOTION_ENABLE_SETTLE_S` (2.0 s) after
`motion_enable(True)` so subsequent `set_mode` / `set_state` calls
don't race the bring-up. Aegis startup orchestration must budget
for this latency.

### Servo-level errors can survive `clean_error()`

**Verified empirically (2026-04-26).** After a prior run where an
unprime fired while joint 6 was still chasing a `set_servo_angle_j`
target (the pre-streaming, one-shot version), the controller latched
a servo-level error on joint 6:

```
servo_error_code, servo_id=6, status=1, code=23
```

The next `prime()` ran, every step returned `code=0`, but the
underlying servo error was still latched. The follow-up
`set_servo_angle_j` then failed with `code=1` (Not Ready) because
joint 6 wasn't actually armed.

Findings:
- `clean_error()` clears **controller**-level errors. It does **not**
  always clear servo-level errors. `clean_warn()` doesn't fix it
  either.
- The bulletproof recovery is **power-cycling the arm**. After power
  cycle, `lite6_cli probe` reports `error_code=0`, `warn_code=0`,
  and prime works again.
- The `servo_error_code, servo_id=N, status=..., code=...` line is
  printed directly by the SDK during `motion_enable(True)`; we can't
  easily suppress it.

`prime()` now reads `arm.error_code` and `arm.warn_code` after the
sequence and raises with a power-cycle hint if either is non-zero,
so this failure mode surfaces during prime instead of as a cryptic
`code=1` on the next motion command.

#### Likely root cause of servo error code 23

Triggered by an abrupt `motion_enable(False)` while a servo loop
was actively tracking an unmet setpoint. The streaming version of
`send_joint_positions` should prevent this since it polls until the
pose lands before unpriming, but **never call `set_servo_angle_j`
once and immediately unprime** -- it's a recipe for latched servo
errors.

(The `code=23` value itself isn't documented in our notes yet --
add the lookup if anyone hits it again.)


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
- `is_radian=True` swaps the joint-angle units to radians (and TCP
  rotation units), but TCP **linear** values remain in mm regardless
  (verified via `position` in the probe — see below).
- TODO: confirm units of joint angles match a known pose (joint 1 at
  +π/2, rest of arm slack) against the teach pendant readout.

## Probing arm config (`lite6_cli probe`)

Pure-read pass over a curated list of SDK properties (no `motion_enable`).
The probe walks `_PROBE_PROPERTIES` in `lite6_cli.py` and prints each via
`getattr(arm, name)`; anything missing falls back to `<no such attr>`.

### Connection banner

The SDK prints a one-liner before our connect-message:

```
ROBOT_IP: 192.168.1.178, VERSION: v1.11.4, PROTOCOL: V1, DETAIL: 6,9,LI1002,DL1000,v1.11.4, TYPE1300: [0, 0]
change protocol identifier to 3
```

Always emitted, no off switch — accept that it's there in any tooling
that connects.

### Property results (firmware v1.11.4)

| property                  | value                                       | notes |
| ------------------------- | ------------------------------------------- | ----- |
| `version`                 | `'6,9,LI1002,DL1000,v1.11.4'`               | comma-joined: axes (6), ?, model (LI1002), drive (DL1000), firmware (v1.11.4). No separate `firmware_version` attr. |
| `firmware_version`        | `<no such attr>`                            | drop from the probe list. |
| `axis`                    | `6`                                         | matches `LITE6_DOF`. |
| `dof`                     | `<no such attr>`                            | drop from the probe list. |
| `is_simulation_robot`     | `False`                                     | real hardware. |
| `state`                   | `5`                                         | observed at connect (pre-prime). State 5 = "stopped"-ish; not in our `_XARM_STATE_*` table. After `prime` it should be `0`/READY -- TODO confirm next probe-after-prime. |
| `mode`                    | `0`                                         | default mode at boot is `0` (motion-plan position), not `1` -- so if a tool relies on mode being 1, it must `set_mode(1)` itself. |
| `error_code` / `warn_code`| `0` / `0`                                   | clean. |
| `cmdnum`                  | `<no such attr>`                            | drop from the probe list. |
| `angles`                  | radians, length-7 (last is the 7-DOF pad)   | matches `get_joint_states` output. |
| `position`                | `[83.7, -0.0, 152.6, -3.14, 0.019, 0.021]`  | TCP pose. **Linear units are mm, rotation units are rad** even with `is_radian=True` -- the flag only swaps rotation units. |
| `tcp_offset`              | zeros (length 6)                            | no TCP offset configured. |
| `world_offset`            | zeros (length 6)                            | base frame == world frame. |
| `tcp_load`                | `[0.0, [0.0, 0.0, 0.0]]`                    | shape: `[mass_kg, [com_x, com_y, com_z]]`. Currently no payload set. |
| `gravity_direction`       | `[0.0, 0.0, -1.0]`                          | gravity in -Z (arm flat-mounted on a table). |
| `joint_speed_limit`       | `[0.001, 3.14159]`                          | **`[min, max]` in rad/s -- global, not per-joint.** Firmware joint maxvel is π rad/s (~180°/s). Same ceiling for all 6 joints. |
| `joint_acc_limit`         | `[0.01, 20.0]`                              | global `[min, max]` in rad/s². 20 rad/s² ceiling. |
| `tcp_speed_limit`         | `[0.1, 500.0]`                              | global `[min, max]` in mm/s. 500 mm/s linear cap. |
| `tcp_acc_limit`           | `[1.0, 500000.0]`                           | global `[min, max]`; units **likely** mm/s² (so 500 m/s² ceiling). Worth re-confirming if it ever matters. |
| `collision_sensitivity`   | `3`                                         | scale 0–5 (0 = off). 3 = mid-sensitivity force-collision detection. |
| `teach_sensitivity`       | `3`                                         | scale 0–5; relevant only in manual-drag teach mode. |
| `self_collision_detection`| `<no such attr>`                            | not exposed as a property on this SDK. Geometric self-collision belongs **upstream** of the driver (see "Self-collision" answer in conversation). Drop from probe list. |
| `motor_enable_states`     | `[1, 1, 1, 1, 1, 1, 0, 0]`                  | 8-element. **Slots 1–6 are the arm joints, slots 7–8 are gripper-related (always `0` in this probe -- gripper not connected to motor enable line).** Note that motors read as enabled even though we never called `motion_enable(True)` -- so this property reflects "powered up" state, not the software motion-enable flag. |
| `motor_brake_states`      | `[1, 1, 1, 1, 1, 1, 0, 0]`                  | same shape. `1` likely = "brake released" given motors are reported enabled. |

### Implications for manor

- **Joint maxvel = π rad/s** is the firmware ceiling. The aegis stack's
  velocity-mode commands should keep well below this — clamp upstream
  to e.g. 1 rad/s as a soft limit.
- **Joint maxacc = 20 rad/s²** is the ceiling. Probably never relevant
  to streaming control, but useful when configuring planned moves.
- **TCP linear maxvel = 500 mm/s, max acc ≈ 500 m/s²** — only relevant
  if we ever expose Cartesian-space commands.
- **No `self_collision_detection` property** — confirms manor must do
  geometric self-collision checks upstream of `Lite6Driver`.
- **`is_radian=True` only swaps rotation units; linear stays mm.** When
  reading `position`, slice-and-convert if any caller needs SI metres.
- **Probe list cleanup**: drop `firmware_version`, `dof`, `cmdnum`, and
  `self_collision_detection` from `_PROBE_PROPERTIES` next time we
  touch the script.

## Sending position commands (`lite6_cli send_joint_positions`)

Uses mode 1 (servo position) and `set_servo_angle_j(angles=...,
is_radian=True)` -- the same SDK call that
`Lite6Driver.write_joint_positions` makes in production, so this
command exercises the real code path.

Each `-jN` / `--jN` flag overrides that joint's target; unspecified
joints default to the **current** angle (read with `get_joint_states`
immediately before the move), so `-j6 0.5` is a single-joint wiggle.

### `set_servo_angle_j` is a streaming setpoint, not a one-shot move

**Verified empirically.** A single call to `set_servo_angle_j` only
makes incremental progress toward the target — the firmware applies
a per-tick step cap, so the servo loop advances by that step and
then settles at an intermediate setpoint short of where you asked
for. To actually reach the target you have to **resend the same
target on every tick** of a streaming loop; the firmware composes
the stream of setpoints into a continuous motion.

This is exactly the contract `Lite6Driver.write_joint_positions` is
built around — Kyber calls it every tick (~500 Hz) with tiny
per-tick deltas. The CLI mirrors this by streaming the target at
100 Hz inside `_stream_to_target` until the measured pose is within
5 mrad (or 5 s elapses).

Implication: **never call `set_servo_angle_j` once and expect the
arm to land at the target.** If you're outside Kyber's loop and need
a one-shot move, either stream the target until settled, or use
mode 0 (`set_servo_angle`) which has built-in trajectory generation.

### Findings

- `set_servo_angle_j` is a streaming setpoint (verified, 2026-04-26).
- TODO: how many ticks does it actually take to settle for a 0.1 rad
  delta at 100 Hz? (The CLI prints this when it returns -- log it.)
- TODO: behavior when target exceeds joint range — error code vs.
  silent clamp vs. fault?
- TODO: per-call return latency — does `set_servo_angle_j` return
  in <1 ms (lets us run the production 500 Hz loop comfortably) or
  is it more like several ms? Run `lite6_cli stream` after a
  `send_joint_positions` and watch what tick rate the streaming loop
  actually achieves.

## Sending velocity commands (`lite6_cli send_joint_velocities`)

Uses mode 4 (joint velocity) and `vc_set_joint_velocity(speeds=...,
is_radian=True, duration=0)`. The script applies the commanded vector
for `--duration` (`-d`) seconds then sends a zero-velocity command;
the zero-out is in a `finally` so an early Ctrl-C still parks the arm.

The SDK's `duration=0` here means "no auto-zero from the firmware" —
we own the lifetime client-side.

### Findings

- TODO: SDK `duration` parameter semantics — confirm `0` means "until
  next command" vs. some default timeout.
- TODO: clamping vs. fault when `speeds[i]` exceeds firmware joint maxvel.
- TODO: deceleration profile after sending zeros — instant, or ramped?
- TODO: minimum / maximum useful `--duration` values (round-trip latency
  floor; firmware watchdogs at the high end?).

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
