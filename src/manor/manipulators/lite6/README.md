# xarm-python-sdk: Lite6 API notes

Empirical notes captured while bringing up the Ufactory Lite6 against
[`xarm-python-sdk`](https://github.com/xArm-Developer/xArm-Python-SDK)
(installed as the `xarm` package). Each section corresponds to one
experiment in `src/manor/manipulators/lite6/lite6_cli.py`. The goal is
to record quirks, return codes, and timing observations that aren't
obvious from the SDK source — so we can lean on this file when wiring
the hardware backends inside aegis.

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
  to keep our own output clean (`lite6_cli.py` does this).

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
- A latched servo error manifests as `arm.error_code=16` at the
  controller level (verified after the in-script post-prime check
  caught it the second run).
- **Soft recovery sometimes works**: cycling `motion_enable(False)`
  → wait → `motion_enable(True)` → re-run prime sequence. This
  effectively re-energises the servos from cold and often clears
  latched errors that `clean_*` can't.
- **The bulletproof recovery is power-cycling the controller**.
  After a real power cycle, `lite6_cli probe` reports `error_code=0`,
  `warn_code=0`, and prime works again. Required when soft recovery
  also fails.
- The `servo_error_code, servo_id=N, status=..., code=...` line is
  printed directly by the SDK during `motion_enable(True)`; we can't
  easily suppress it.

`prime()` now does the standard sequence, checks
`arm.error_code` / `arm.warn_code`, attempts a soft recovery
(`_try_soft_recover`) on failure, and re-checks. Only raises with
a power-cycle hint if both attempts left errors latched -- so this
failure mode surfaces during prime instead of as a cryptic `code=1`
on the next motion command.

#### Likely root cause of servo error code 23

Triggered by an abrupt `motion_enable(False)` while a servo loop
was actively tracking an unmet setpoint. The current `send_jp`
implementation prevents this by polling `arm.angles` until the
pose lands before returning (and only then does the caller unprime),
but **never call `set_servo_angle_j` once and immediately
unprime/disable motors** -- it's a recipe for latched servo errors.

(The `code=23` value itself isn't documented in our notes yet --
add the lookup if anyone hits it again.)

### `code=1` ("Not Ready") on the first SDK call

If a fresh connection's first SDK call (e.g. `clean_warn`) returns
`code=1`, the controller is refusing all commands. We can't recover
from this in software -- once the controller is in this state every
command we send (including `clean_*` and `motion_enable(False)`)
gets bounced with `code=1`. Likely causes:

1. **E-stop engaged.** Physical button on the controller or teach
   pendant is depressed. Release before retrying.
2. **Manual-drag / teach mode active.** Whatever puts the arm in
   manual-drag also makes it ignore programmatic commands.
3. **Latched servo error wedged the state machine.** Even with
   `clean_error` / `clean_warn` available, the controller-level
   state machine can refuse to accept them.

Resolution path:

1. Release the e-stop if engaged.
2. Power-cycle the controller. **Wait 10 s with power off** before
   turning back on -- capacitors need to fully discharge or the
   same servo state can re-latch.
3. After boot, run `lite6_cli probe` first to verify clean state
   (`error_code=0`, `warn_code=0`) before trying any motion command.

The `_check` helper in `lite6_cli.py` now appends this guidance to
its `RuntimeError` when it sees `code=1`, so future failures surface
with the actionable steps in the message itself.


### Current `lite6_cli` lifecycle (verified on hardware)

The `lite6_cli` prime/unprime/disconnect split below is what we
actually run. It deliberately diverges from the older
`Lite6Driver.prime()` sequence in a couple of places (notably: prime
moves to a known-clear pose, and `unprime` does NOT call
`motion_enable(False)`); the production driver should adopt these
once we're done characterising mode 1.

**`prime(mode=...)`** — bring the arm up + move to a known-clear pose:

1. `clean_warn()` + `clean_error()` — wipe any latched faults.
2. `motion_enable(enable=True)` — energize motors (audible relay click).
3. Sleep `_MOTION_ENABLE_SETTLE_S = 2.0 s` — servos lock onto encoder pose.
4. `set_mode(0)` — **always** activate mode 0 (motion-plan position) so we can
   use `set_servo_angle` for the move-to-PRIME step.
5. `set_state(0)` — READY.
6. Check `arm.error_code` / `arm.warn_code`; on non-zero, attempt soft
   recovery (`motion_enable` toggle + replay) and re-check; raise with
   power-cycle hint if still latched.
7. `set_servo_angle(angle=PRIME, wait=True)` — move to the
   `Lite6JointConfiguration.PRIME` pose. Done in mode 0 so we don't
   depend on mode 1 (the call we're characterising) for safe positioning.
8. If the caller's `mode` isn't 0, switch to it now via
   `STOP / set_mode(mode) / READY`.

**`unprime`** — return to ZERO and halt, but stay energized:

1. `STOP / set_mode(0) / READY` (always; idempotent if we're already there).
2. `set_servo_angle(angle=ZERO, wait=True)`.
3. `set_state(STOP)`.

Critically does NOT call `motion_enable(False)` or `disconnect()` —
motors stay energized so the next prime skips the audible-click /
encoder-relock cycle, and the TCP session stays open so we skip the
SDK re-handshake. See "set_state vs motion_enable vs disconnect"
below for the reasoning.

**`disconnect`** — full teardown when truly done:

1. `set_state(STOP)`.
2. `motion_enable(enable=False)`.
3. `disconnect()`.

Run this at end-of-session, before powering down, or when the arm
will be idle long enough that energized servos would warm up.

### `wait_move` bails out when `arm.mode` is stale (heartbeat-cached lag)

**Verified by reading SDK source + reproduced on hardware.** This
bites anyone who calls `set_servo_angle(wait=True)` shortly after a
mode switch, and silently breaks `unprime`-style "move then stop"
sequences. Worth understanding once.

`xarm/x3/base.py:wait_move` (the function `set_servo_angle(wait=True)`
calls internally) has this check at the top of every iteration:

```python
if self.mode != 0 and self.mode != 11:
    return 0
```

It bails out **with success** when `arm.mode` doesn't match mode 0
(or the rarely-used 11). The catch: `arm.mode` is updated from the
SDK's heartbeat report (~5 Hz), so it lags the most recent
`set_mode()` call by up to ~200 ms. So if you call:

```python
arm.set_mode(0)                                  # firmware now in mode 0
arm.set_state(0)                                 # READY
arm.set_servo_angle(angle=..., wait=True)        # wait_move sees stale arm.mode -> returns 0 instantly
arm.set_state(4)                                 # cancels the just-queued motion before firmware executes it
```

…your `wait=True` becomes effectively `wait=False`, and the trailing
`set_state(STOP)` cancels the queued trajectory.

`prime()` happens to hide this: the 2 s `_MOTION_ENABLE_SETTLE_S`
sleep before the move-to-PRIME step gives the heartbeat plenty of
time to catch up. `unprime` had no such accidental delay, which made
the bug surface only there.

**Workaround:** poll `arm.mode` after every `set_mode()` until it
reflects the new mode (or short timeout). `_switch_mode` in
`lite6_cli.py` does this with `_MODE_REPORT_SETTLE_S = 1.0 s`.

### `set_state(STOP)` vs `motion_enable(False)` vs `disconnect()` — the practical diff

| call | what it does | reversal cost |
| ---- | ------------ | ------------- |
| `set_state(state=4)` (STOP) | Software state-machine flip. Halts pending motion, makes the controller reject new motion commands until READY. **Motors stay energized; brakes stay released; servos still hold position via active control.** | `set_state(0)` is essentially instant. |
| `motion_enable(enable=False)` | Hardware-level motor de-energize. Servos lose power, mechanical brakes engage. | `motion_enable(True)` triggers the audible click + ~2 s encoder-relock on every cycle. |
| `disconnect()` | Closes the TCP socket. **No effect on motor or controller state.** | `XArmAPI(...)` reconnects with a full SDK init handshake. |

**Practical implication for tooling:** Across rapid CLI invocations,
prefer `set_state(STOP)` over `motion_enable(False)` between
commands — motors stay energized, no 2 s relock. Only run a full
`STOP → motion_enable(False) → disconnect()` teardown at end of
session (or before powering down). `lite6_cli` reflects this:
`unprime` does only `set_state(STOP)`; the dedicated `lite6_cli
disconnect` command does the full teardown.

### xarm mode constants

Captured in `lite6_cli.py` as `XArmMode(IntEnum)`:

| value | name             | use                                                   |
| ----- | ---------------- | ----------------------------------------------------- |
| `0`   | `POSITION`       | motion-plan-style API (`set_position`, `set_servo_angle`) |
| `1`   | `SERVO_POSITION` | low-latency joint streaming via `set_servo_angle_j`   |
| `2`   | `MANUAL`         | joint teaching / manual drag (gravity-comp, drag by hand) |
| `4`   | `VELOCITY`       | `vc_set_joint_velocity`                                |

(There are other modes — these are the four manor uses.)

### xarm state constants

Captured in `lite6_cli.py` as `XArmState(IntEnum)`:

| value | name    |
| ----- | ------- |
| `0`   | `READY` |
| `4`   | `STOP`  |

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
| `state`                   | `5`                                         | observed at connect (pre-prime). State 5 = "stopped"-ish; not in our `XArmState` enum. After `prime` it should be `0`/READY -- TODO confirm next probe-after-prime. |
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

## Sending position commands (`lite6_cli send_jp`)

Uses mode 1 (servo position) and `set_servo_angle_j(angles=...,
is_radian=True)` -- the same SDK call that
`Lite6Driver.write_joint_positions` makes in production, so this
command exercises the real code path.

Each `-jN` / `--jN` flag overrides that joint's target; unspecified
joints default to the **current** angle (read with `get_joint_states`
immediately before the move), so `-j6 0.5` is a single-joint wiggle.

### `set_servo_angle_j` semantics — what's actually true

Read carefully because the early experiments led to a couple of
wrong inferences that have since been corrected.

**SDK-side (Python wrapper, verified by reading source):**

- `xarm/wrapper/xarm_api.py` and `xarm/x3/xarm.py` show
  `set_servo_angle_j` does **no per-call delta capping** in Python.
  It validates joint range (returns `OUT_OF_RANGE` if violated),
  converts to radians, and forwards to the controller's binary
  `move_servoj` command.
- The wrapper's own docstring says: *"Set the servo angle, **execute
  only the last instruction**, need to be set to servo motion mode."*
  Translation: re-sending the same target is benign — the firmware
  treats every call as a fresh target replace.
- `Lite6Driver.write_joint_positions` (production driver) does not
  interpolate. It just forwards `set_servo_angle_j(angles=..., is_radian=True)`.
  Kyber may send a far-away Cartesian-IK joint solution in a single tick,
  which means the firmware itself must be capable of accepting
  far-away targets and servoing toward them under the joint speed
  bound — there is no requirement to interpolate before sending.

**Firmware-side (verified empirically, 2026-04-28):**

- A single `set_servo_angle_j` call does **not** instantly move the
  arm to the target. The arm progresses toward it over time, bounded
  by `joint_speed_limit` (`[0.001, π]` rad/s, see probe).
- **One call is enough.** No per-tick step cap, no continuous
  streaming required: the firmware servoes to the latched target on
  its own under the velocity bound. Verified with two trials from
  PRIME pose:
  - `j6` from PRIME → `+0.5` rad: target reached in ~0.4 s with one
    SDK call.
  - `j6` from PRIME → `-1.5` rad: target reached in ~0.6 s with one
    SDK call.
  No additional `set_servo_angle_j` calls were issued during either
  trial; `arm.angles[5]` walked smoothly to the target and held.
- This resolves the earlier ambiguity ("velocity-limited servo
  chasing a latched target" vs. "per-tick step + retarget every
  call") in favour of the former. So `Lite6Driver.write_joint_positions`
  doesn't actually need a streaming loop to *complete* a motion --
  Kyber's 500 Hz cadence is for **smooth retargeting** (each tick
  replaces the latched target with the next IK solution), not for
  keeping the firmware servoing.

**Why "code=1 on the second streaming call" doesn't mean "target rejected":**

`xarm/x3/base.py:_check_code` re-maps the firmware return on move
commands:

```python
if is_move_cmd:
    if code in [0, WAR_CODE]:
        if self.arm_cmd.state_is_ready:
            return 0
        else:
            return STATE_NOT_READY  # value: 1
    ...
```

So even if the underlying TCP `move_servoj` returns success, the SDK
overrides it to `code=1` ("Not Ready") whenever
`arm_cmd.state_is_ready` is false at that moment. A latched servo
fault triggered *between* two streaming calls (e.g. the first call
moved the arm into a self-collision) flips `state_is_ready` to
false; the next call then surfaces as `code=1` even though nothing
about the target itself was wrong.

This invalidates the earlier theory that "spamming the same far
target causes the firmware to reject subsequent calls" — the
rejection is correlated with a fault, not with the streaming pattern.

### Findings

- `set_servo_angle_j` accepts far-away targets without per-call
  interpolation on the SDK side (verified by reading source,
  2026-04-26).
- A `code=1` on a move command means "state not ready right now",
  not "this target is invalid". Always check `arm.error_code` /
  `arm.warn_code` after such a failure to find the real cause.
- **One call + convergence poll is enough.** `lite6_cli send_jp`
  issues a single `set_servo_angle_j` then polls `arm.angles` at
  ~20 Hz until within `_SETTLE_TOLERANCE_RAD = 5e-3` rad of the
  target. No streaming loop on the client side; firmware servoes
  under the velocity bound on its own. (Verified 2026-04-28 -- see
  the firmware-side notes above.)
- TODO: behavior when target exceeds joint range — error code vs.
  silent clamp vs. fault?
- TODO: per-call return latency — does `set_servo_angle_j` return
  in <1 ms (lets us run the production 500 Hz loop comfortably) or
  is it more like several ms? Run `lite6_cli stream` after a
  `send_jp` and watch what tick rate the streaming loop actually
  achieves.

### Self-collision: firmware does NOT prevent it (incident, 2026-04-26)

While experimenting with `send_jp -j6 0` to swing joint
6 back through its range during streaming bring-up, the arm
**self-collided**. The first call appeared to begin the motion;
subsequent calls returned `code=1`. After investigation
(see "Why 'code=1' doesn't mean 'target rejected'" above) the
likely sequence was:

1. The first `set_servo_angle_j` call accepted the target.
2. The servo loop drove joint 6 toward the new pose along a path
   that intersected another link.
3. Force-collision detection (`collision_sensitivity=3`, see probe)
   tripped a fault, latching servo errors and flipping
   `state_is_ready` → false.
4. The streaming loop's next call surfaced as `code=1`.

Confirms the answer to the earlier question "does the robot do
self-collision checks internally?" — **no, not geometric
self-collision**. The firmware has *force*-collision detection
(detects unexpected torque on a joint) but no geometric model of
the arm to refuse a path before motion starts. Geometric
self-collision must be enforced **upstream** of the driver
(planner / Kyber / IK constraint), as also noted in the probe
implications above.

Recovery from this state required power-cycling the controller
(the latched servo error survived `clean_error` and soft recovery,
matching the "servo-level errors can survive `clean_error()`"
section above).

## Sending velocity commands (`lite6_cli send_jv`)

Uses mode 4 (joint velocity) and `vc_set_joint_velocity(speeds=...,
is_radian=True, duration=0)`. The script applies the commanded vector
for `--duration` (`-d`) seconds then sends a zero-velocity command;
the zero-out is in a `finally` so an early Ctrl-C still parks the arm.

The SDK's `duration=0` here means "no auto-zero from the firmware" —
we own the lifetime client-side.

### Findings

- **Basic flow works on hardware** (verified 2026-04-27): set the
  velocity via `vc_set_joint_velocity(speeds=..., duration=0)`, sleep
  for `--duration`, then send zero speeds. The arm tracks the
  commanded velocity and stops when the zero comes through.
- Mode-switch from mode 0 (prime's default) to mode 4 happens inside
  `prime()` after the move-to-PRIME via `_switch_mode` — same
  arm.mode-poll fix as everywhere else.
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
  position / state? (driver currently returns `None` for both EE
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
