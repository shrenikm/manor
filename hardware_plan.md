# Hardware bring-up plan

## Scope

Track everything we need to verify, wire up, or revisit before the
aegis stack runs against the real Lite6 + camera. Sim mode runs today;
this doc is the next-task punch list for hardware. Treat each section
as a checklist to walk top-to-bottom on first power-up.

The aegis hardware-mode process layout (`metis` + `kylos` + `helios`)
is already in place — this doc is about the things behind those
boundaries (driver, SDK quirks, network, real cameras, safety) that
sim mode doesn't exercise.

---

## 1. xarm SDK fit assessment (`xarm-python-sdk==1.17.3`)

Already installed as a hard dependency. Inspected against `Lite6Driver`
in `src/manor/manipulators/lite6/driver.py`. **The driver as written
is API-compatible with the current SDK.** Every method we call exists
with a signature that matches our usage.

### Methods we call — all confirmed present

| Driver call | SDK signature (1.17.3) | Notes |
|---|---|---|
| `set_servo_angle_j(angles, is_radian=True)` | `set_servo_angle_j(angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs)` | `speed`/`mvacc`/`mvtime` are documented as **reserved** in the docstring — not used in the protocol. Just `angles` is real. **No velocities are required for joint position commands** despite faint memory of the contrary. |
| `vc_set_joint_velocity(speeds, is_radian=True, duration=0)` | `vc_set_joint_velocity(speeds, is_radian=None, is_sync=True, duration=-1, **kwargs)` | Pure velocity API — symmetric to the position one. `duration=0` means "always effective, no auto-stop". |
| `get_joint_states(is_radian=True)` | returns `(code, [position, velocity, effort])` | Each inner list is **7-long regardless of arm DOF**. Our test mock simulates this; the driver slices `[:LITE6_ARM_DOF]`. We use `position` and `velocity`, ignore `effort`. |
| `open_lite6_gripper()` / `close_lite6_gripper()` / `stop_lite6_gripper()` | each takes `sync=True` | See `sync` caveat below. |
| `set_vacuum_gripper(on)` | `set_vacuum_gripper(on, wait=False, timeout=3, delay_sec=None, sync=True, hardware_version=1)` | See `hardware_version` caveat below. |
| `clean_error()`, `motion_enable(enable=True)`, `set_mode(mode)`, `set_state(state)`, `emergency_stop()` | all present, signatures unchanged | — |

### Mode / state constants are still valid

- `set_mode(1)` = servo motion mode — required before `set_servo_angle_j`. ✓
- `set_mode(4)` = joint velocity control mode — required before `vc_set_joint_velocity`. ✓
- `set_state(0)` = motion/ready. ✓
- `set_state(4)` = stop. ✓

The SDK now exposes more modes (5 = cartesian velocity, 6/7 = online TOPP)
that we don't use today.

### Caveats / firmware gates

1. **Firmware-version requirements.** Each method has a minimum:
   - `vc_set_joint_velocity` ≥ 1.6.9 (and `duration` arg ≥ 1.8.0)
   - `get_joint_states` ≥ 1.9.0
   - `open_lite6_gripper` / `close_lite6_gripper` / `stop_lite6_gripper` ≥ 1.10.0
   - `sync` arg on the lite6-gripper methods ≥ 2.4.101

   **Action:** before first power-up, run `arm.get_version()` and confirm firmware
   on the controller is ≥ 2.4.101 (the highest watermark above). If it's older,
   either upgrade the controller or drop the `sync=False` plan from caveat #2.

2. **Gripper `sync` default = `True`.** Currently we use the SDK default, which
   means gripper commands queue **behind** any pending motion in the controller's
   queue. For our streaming-control pattern (`set_servo_angle_j` is sub-millisecond
   and we want gripper commands to fire immediately), `sync=False` is the right
   choice. One-line change in `Lite6Driver._send_gripper_command` /
   `_send_gripper_stop`. **Action:** flip to `sync=False` and verify on hardware.

3. **`set_vacuum_gripper(hardware_version=1)`.** Default `1` = plug-in connection;
   `2` = contact connection. We don't surface this — fine for the current arm
   but worth bubbling into `Lite6Driver` config if we ever swap tools.

4. **Constructor param naming.** `XArmAPI(port=..., is_radian=...)` — `port` is
   awkwardly named but accepts the IP string. We pass `port=self.ip` correctly.

5. **`vc_set_joint_velocity(is_sync=True)` default.** All joints accelerate /
   decelerate together with synchronized timing. Probably what we want; flagging
   if independent per-joint velocity profiles ever matter.

6. **`is_radian` semantics.** We pass `is_radian=True` to the constructor (sets
   the default) **and** redundantly to every call. Belt-and-suspenders, harmless.

### SDK features we don't use yet (worth knowing exist)

- **Tool I/O:** `set_tgpio_digital(pin, value)`, `get_tgpio_digital(pin)` — for
  custom end-effectors, sensors, lights mounted on the tool flange.
- **F/T sensor:** `get_ft_sensor_data()` and family — only relevant if the
  optional force-torque module is fitted.
- **Cartesian streaming:** `set_servo_cartesian()` — cartesian-space equivalent
  of `set_servo_angle_j`, would let us stream EE poses if Kyber's diff-IK lives
  outside the arm.
- **Online trajectory planning modes** (mode 6/7) — server-side TOPP with
  position/velocity targets. Probably not what we want (we want closed-loop
  control from Kyber), but useful for canned moves like "go home".
- **Diagnostics:** `get_version()`, `get_state()`, `get_err_warn_code()`,
  `get_temperature()`, `get_voltages()` — surface in `Lite6Driver.health_check()`
  and feed into `aegis status` for at-a-glance hardware health.

---

## 2. `Lite6Driver` — status & verification

### What's already in place

- `prime()` does the full init sequence: `clean_error` → `motion_enable(True)`
  → `set_mode(servo_position)` → `set_state(ready)`.
- `unprime()` does `set_state(stop)` → `motion_enable(False)`, then drops the
  SDK handle. Tolerates being called without a prior `prime`.
- `_check(ret_code, op)` raises `Lite6DriverError` with the failing op name
  if any SDK call returns non-zero, **and triggers `emergency_stop` first**.
  This is the safety hatch.
- xarm SDK is wrapped in `contextlib.redirect_stdout` at import time so the
  `SDK_VERSION: 1.17.3` banner doesn't spam aegis CLI invocations.

### Things to verify on real hardware

1. **End-to-end command/state round-trip.** Smallest possible test:
   `prime()` → `read_joint_positions()` → assert shape is `(6,)` with sane
   values → `unprime()`. No motion.

2. **Move test (slow, single joint).** Set mode to servo_position, send a small
   `set_servo_angle_j` delta, read back, confirm the arm moved to the target
   within tolerance. Use the joint with the smallest reach risk first.

3. **Velocity-control test.** Switch to mode 4, send small velocities for a
   bounded duration (e.g. `vc_set_joint_velocity(speeds=[0.05, 0, ...], duration=0.5)`),
   confirm motion stops when duration elapses or when zeros are sent.

4. **Mode switch behavior.** What happens if we call `set_servo_angle_j` while
   in mode 4? Or `vc_set_joint_velocity` while in mode 1? Expected: SDK returns
   non-zero, `_check` triggers emergency stop. Verify this is what actually
   happens before we trust `_check` as the safety net.

5. **Gripper.** Verify the `sync=False` change (caveat #2) actually executes
   immediately and doesn't wait for queued joint moves to finish.

6. **EEF read-back.** `read_eef_positions` / `read_eef_velocities` currently
   return `None` because the SDK doesn't expose gripper position. Verify this
   is still true; if any newer SDK method exposes gripper state, plumb it.

7. **Connection lifecycle robustness.** `prime` → kill the kylos process
   mid-motion → restart kylos → `prime` again. Does the arm cleanly accept the
   new connection? Are residual errors cleared by `clean_error()`?

8. **Concurrency.** What does the SDK do if `set_servo_angle_j` is called
   while a previous one is still executing? (Should be fine — that's the
   streaming model — but verify under our 100Hz Talos publish rate.)

---

## 3. Network & connection

- **Default IP:** `192.168.1.178` (from the deprecated codebase, hardcoded
  as `_LITE6_DEFAULT_IP` in `driver.py`). **Action:** confirm against the
  controller's actual IP on first boot — the controller has a sticker/menu
  with its address.
- **Subnet:** the lab machine running aegis must be on the same subnet
  (typical `192.168.1.0/24`). Either set up a dedicated NIC or use the
  controller as the only device on a separate network.
- **Latency budget.** The `vc_set_joint_velocity` call rate is bounded by
  the round-trip time over Ethernet plus SDK serialization. Measure with
  a tight loop of `get_joint_states` and check that we can hold 200 Hz
  (Talos's default). If not, drop `talos_config.publish_frequency_hz`
  to whatever is sustainable.
- **Reconnect.** If the network drops, the SDK's behavior (does it retry?
  raise? hang?) needs documentation. Test by physically unplugging the
  Ethernet cable mid-run.
- **Per-instance config.** The IP is currently a constructor arg on
  `Lite6Driver` but isn't surfaced in `lite6_default.yaml`. **Action:**
  add `lite6_driver_config: { ip: ... }` (or similar) to the YAML and
  thread it through, so deployments don't need a code edit to swap arms.

---

## 4. Hardware-mode aegis layout

For reference — three processes, no surprises:

```
metis (LCM)  ←→  kylos (LCM + direct Kyber↔Talos↔Lite6Driver)
                 helios (LCM, real camera SDK)
```

- `kylos` runs the same `Kyber` + `Talos` LeafSystems as gylos, but:
  - Talos's backend is `HardwareManipulatorBackend` wrapping `Lite6Driver`.
  - There's no `Gaia`, no `GaiaAdvancer`, no inner simulator.
  - Talos's publish rate becomes the hardware control-loop rate.
- `helios` runs alone with `HardwareSensorBackend` — see §5.

Both `kylos` and `helios` start their backends in `try / finally` blocks so
the driver / camera SDK gets `unprime()`d / closed even on signal-driven exit.
**Verify this on real hardware** — a botched cleanup leaves the arm
servo-locked and refusing the next prime.

---

## 5. Helios hardware path (cameras)

`HardwareSensorBackend` in `helios/hardware_backend.py` is a structural
stub. `read_rgb` / `read_depth` return empty frames with fresh timestamps.

### Decisions to make before first run

1. **Pick the camera.** The bundled `HardwareSensorBackendConfig` has fields
   for `serial_number` and resolution but no SDK choice. RealSense D4xx is the
   most likely first target (well-supported, has `pyrealsense2` Python bindings).
   Orbbec is the alternative if a different camera is on hand.

2. **Camera mounting.** Where on the workspace is the camera mounted? Eye-in-hand
   (on the EEF) vs. eye-to-hand (fixed in the world)? The mount choice changes
   the calibration story.

3. **Calibration.** Need an extrinsic calibration pipeline (camera frame → robot
   base frame). For eye-to-hand: ArUco / Charuco target on the EEF is the
   simplest. Need to build a calibration script under `scripts/`.

4. **Wiring `start()` / `stop()`.** Open the SDK pipeline in `start`, close in
   `stop`. Match the pattern in `talos/hardware_backend.py`.

5. **Frame format conversion.** SDK frames are typically a vendor-specific
   buffer; convert to `RGBImageData` / `DepthImageData` (numpy arrays of the
   right dtype/shape). Define a helper in `hardware_backend.py` so the
   `read_rgb` / `read_depth` body stays one-liner.

6. **Frame rate vs. publish rate.** `helios_config.publish_rgb_frequency_hz` is
   30 Hz. The camera's native frame rate may be 30 / 60 / 90. We probably want
   helios's publish rate ≤ camera frame rate so we never publish stale frames.
   Add a guard.

7. **Latency / timestamp.** Use the camera's hardware timestamp (most SDKs
   expose it) rather than wall-clock-at-receive, so policy training has accurate
   sensor timestamps.

---

## 6. Timing & realtime considerations

### Clock pacing

- **`metis` process:** `target_realtime_rate=1.0`. Policy ticks happen at
  `metis_config.publish_frequency_hz` (10 Hz) wall-clock.
- **`kylos` process:** runs at `target_realtime_rate=1.0`. Talos's publish
  rate (200 Hz target) is the hardware control loop rate. Kyber's controller
  rate (500 Hz) is also wall-clock.
- **`helios` process:** runs at `target_realtime_rate=1.0`. Camera publish
  rate matches the YAML.

These are all wall-clock paced because hardware *is* wall-clock paced.
Sim's "lockstep" mode discussed elsewhere doesn't apply here — the real
arm doesn't pause.

### Linux real-time tuning

The kylos process is what hits the SDK at 200 Hz. To minimize jitter:

- **Process priority:** start `kylos` with `chrt -f 50` (SCHED_FIFO,
  priority 50) or via systemd's `CPUSchedulingPolicy=fifo`. Without this,
  `set_servo_angle_j` calls can be preempted by background tasks and
  the arm will see periodic stalls.
- **CPU affinity:** pin kylos to a dedicated core (`taskset -c 2`).
- **Disable CPU frequency scaling** on the dedicated core (set governor
  to `performance`).
- **isolcpus** kernel arg if we get serious about determinism.

These are followups for *after* basic correctness is verified. Don't
optimize before measuring.

### Latency measurement

Add a one-shot script that:
1. Calls `set_servo_angle_j` with a tiny delta.
2. Polls `get_joint_states` until the position matches.
3. Records the wall-clock delta.

This gives us baseline command-to-execution latency. Track over time
to catch network or controller-firmware regressions.

---

## 7. Safety & error recovery

### What's wired today

- `Lite6Driver._check(ret_code, op)` calls `emergency_stop()` and raises
  `Lite6DriverError` on any non-zero SDK return code.
- Talos's `HardwareManipulatorBackend.stop()` calls `driver.unprime()`.
- The kylos runner has a `try/finally` around `advance_until_signal`
  that calls `backend.stop()`.

### Holes to plug

1. **No software watchdog.** If kylos crashes mid-motion, the arm might
   continue executing the last `vc_set_joint_velocity` until `duration`
   expires (default 0 = forever). **Action:** consider always passing a
   small `duration` (e.g. 0.1 s) on velocity commands so the arm
   auto-stops if our process goes silent. Trades safety for needing to
   re-publish at ≥ 10 Hz to avoid stuttering — which we already do at 200 Hz.

2. **No physical e-stop integration.** The arm has a hardware e-stop
   button on the controller. After it's pressed, our `prime()` sequence
   should clear errors and re-enable motion — verify this works via
   `clean_error` → `motion_enable(True)` → `set_state(0)`.

3. **No collision/limit checking on outgoing commands.** Today we forward
   whatever `JointPositions` the upstream controller sends. The SDK does
   internal joint-limit clamping but it's defensive — we should also
   clip in `Lite6Driver` against `manipulator_model` limits as a
   second line of defense.

4. **No "go home" recovery.** After an error, the operator needs a way
   to send the arm to a known safe pose. Add a `Lite6Driver.go_home()`
   that uses `set_servo_angle` (the trajectory-planned version, not the
   streaming `_j` version) to drive to the URDF default pose.

5. **Logging of SDK calls.** When something goes wrong on hardware, we
   need a record of what was sent and what came back. Add structured
   logging in `Lite6Driver._check` (and on the `write_*` methods)
   gated by an env var, so we don't spam in the steady state.

---

## 8. Gripper continuous-state asymmetry

This is intrinsic to the hardware — flagging so we don't pretend it
isn't there.

**Hardware:** the parallel-gripper SDK is binary (`open` / `close` / `stop`);
no API to set finger position. Vacuum is on/off. So:

- Continuous `EEFPositions` from upstream → `Lite6Driver.write_eef_positions`
  thresholds to open/close (~4 mm threshold).
- `Lite6Driver.read_eef_positions` returns `None` because the SDK doesn't
  expose gripper state.

**Sim:** the parallel-gripper URDF *does* model the two prismatic finger
joints, and `Gaia`'s `InverseDynamicsController` controls them as
continuous joints. Fingers can be at any position in `[0, 0.008]` m.

**Reconciliation:** the cleanest fix is to have
`HardwareManipulatorBackend.read_eef_state` return the
**last commanded thresholded position** (instead of zeros), so downstream
consumers see a stable "open=0.008m, closed=0m" reading even though
it's not measured. Easy bit of state to thread:

- Add `_last_eef_command: EEFPositions | None` to `HardwareManipulatorBackend`.
- Set on each `send_command`.
- Return on `read_eef_state` instead of zeros.

---

## 9. Operational testing protocol (first power-up)

Run these in order. Stop at the first thing that fails — debug it before moving on.

1. **Network ping.** `ping 192.168.1.178` — confirm the controller is reachable.
2. **SDK connect (no motion).** Run `python -c "from xarm.wrapper import XArmAPI; arm = XArmAPI('192.168.1.178', is_radian=True); print(arm.get_version())"` — confirms SDK can reach the controller and firmware version.
3. **Driver smoke test.** Standalone Python: `Lite6Driver.prime()` → `read_joint_positions()` → `unprime()`. No motion expected.
4. **Single-joint move.** Send a small `set_servo_angle_j` delta (e.g. 0.05 rad on joint 1). Visually confirm the arm moves to the target.
5. **Gripper toggle.** Open → close → stop. Visually confirm.
6. **Standalone hardware mode runners.** `aegis run kylos` only (no metis yet); confirm `AEGIS_PROPRIOCEPTION` lands on LCM at the configured rate.
7. **Helios standalone.** `aegis run helios` only; confirm `AEGIS_RGB_IMAGE` / `AEGIS_DEPTH_IMAGE` land on LCM.
8. **Full hardware loop.** `aegis run kylos && aegis run helios && aegis run metis`. Zero-velocity policy (already configured) — arm should stay still while LCM channels carry traffic.
9. **Gentle motion.** Swap metis policy to something that emits a tiny non-zero velocity for a single joint; confirm the arm tracks it.
10. **Failure injection.** Kill kylos mid-motion. Verify the arm halts (or at least stops accepting new commands), and that `aegis run kylos` after recovery succeeds.

---

## 10. Open questions / followups (not blocking initial bring-up)

- IP address per-deployment config in YAML (vs. `_LITE6_DEFAULT_IP` constant).
- Camera SDK choice (RealSense vs. Orbbec) and config model.
- Camera extrinsic calibration pipeline.
- `health_check()` method on `IManipulatorDriver` returning version /
  errors / temperatures — surface in `aegis status`.
- `go_home()` method for post-error recovery.
- Joint-limit clipping in `Lite6Driver.write_joint_positions`.
- Auto-`duration` on `vc_set_joint_velocity` as a software watchdog.
- Last-commanded-thresholded EEF readback for hardware (§8).
- Switch lite6 gripper calls to `sync=False` (§1 caveat 2).
- Latency benchmark script under `scripts/`.
- Linux real-time tuning playbook (process priority, CPU pinning).

---

## Summary

The xarm SDK fits cleanly with `Lite6Driver` as written — no breaking
incompatibilities. The work between here and a running real arm is:

1. **Network + firmware confirmation** (§3, §1 caveat 1).
2. **Camera SDK wiring** (§5) — biggest single chunk; the helios stub needs
   a real implementation.
3. **Driver hardening** — sync=False on grippers, IP in YAML, last-commanded
   EEF readback, `health_check`, joint-limit clipping (§7, §8, §10).
4. **Operational protocol walkthrough** (§9) on the actual robot.

Sim mode work continues in parallel; nothing in the sim path needs to
change for hardware to come up.
