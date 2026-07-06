# manor.manipulators.rebot_b601_dm

Notes on the Seeed Studio reBot Arm B601 DM and how manor drives it. The arm is 6-DOF on Damiao CAN motors
(DM-J4340 on joints 1-3, DM-J4310 on joints 4-6) plus an actuated parallel gripper (a seventh DM-J4310).
All seven motors sit on one CAN bus behind a Damiao serial bridge (`/dev/ttyACM0`, 921600 baud), spoken to
via the `motorbridge` pip package.

These findings come from reading the vendor stack end to end, since getting the gripper control wrong
physically breaks the arm (see below):

- [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) — the low-level SDK (JointGroup
  architecture, mode sequences, gains).
- [reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) — the ROS2 node
  (state machine, gripper services, gravity compensation).
- The official [LeRobot integration](https://huggingface.co/docs/lerobot/main/en/rebot_b601)
  (`rebot_b601_follower` / `rebot_102_leader` in the lerobot repo) — teleop and the definitive gripper
  safety pattern.

## Motor control modes

The DM motors expose four firmware modes (motorbridge `Mode`):

- `POS_VEL` — cascaded position/velocity loop inside the motor, gains in registers (RID 25/26 velocity
  loop, 27/28 position loop; vendor gains: 4340P `(0.0125, 0.004, 150.0, 0.5)`, 4310
  `(0.0008, 0.002, 50.0, 1.0)`). Each command carries a velocity limit. This is what the vendor uses for
  arm position control and what our driver streams joint positions with.
- `MIT` — impedance command `(p_des, v_des, kp, kd, tau_ff)`; torque ≈ kp·err + kd·verr + tau_ff, bounded
  by the motor's max torque. Soft gains = compliance. Vendor endpos control uses kp 120 / kd 8 (4340) and
  kp 18 / kd 2 (4310); the LeRobot teleop follower uses much softer kp 45 / kd 12 and kp 8-9 / kd 1.
- `VEL` — velocity mode; our driver uses it for joint-velocity streaming.
- `FORCE_POS` — position control with a per-command torque ceiling: `send_force_pos(pos, vlim, ratio)`
  where ratio is a fraction of max motor torque. The gripper's only safe mode.

## THE GRIPPER IS 3D PRINTED — TORQUE MUST BE CAPPED

Plain position control (POS_VEL, or MIT with meaningful gains) applies full motor torque when the fingers
meet an object, and this has physically broken the printed gripper linkage during calibration. The vendor's
LeRobot integration drives the gripper exclusively in `FORCE_POS` with `gripper_torque_ratio = 0.07` — the
gripper grips at 7 percent of max motor torque, which holds objects fine and cannot damage the linkage.

Manor enforces this in layers:

- `RebotB601DmDriver` routes every EE write through `send_force_pos` with the configured
  `gripper_torque_ratio`; there is no code path that position-controls the gripper.
- `RebotB601DmDriverConfig.gripper_torque_ratio` is required (every hardware YAML declares it consciously)
  and its validator refuses values above `REBOT_B601_DM_GRIPPER_TORQUE_RATIO_MAX` (0.2).
- The helpers re-clamp the ratio defensively at the send site, and the CLI's `--torque-ratio` option is
  bounded the same way.

Gripper geometry: motor 0 rad = fully closed (fingertips touching), about -5 rad = fully open. The EE-level
width in metres maps linearly onto that motor range (`gripper_width_to_motor_rad`), width 0 to 0.143 m.

## Zeroing / calibration

The motors have no persistent calibration; the vendor tooling re-zeroes against whatever pose the arm holds
at setup. The canonical zero is the vendor home pose: shoulder and forearm horizontal, gripper pointing
forward and fully closed (identical to the URDF q = 0). Use `rebot_b601_dm zero` with the arm physically
held there, then verify with `rebot_b601_dm stream --passive`. Joints 2 and 3 sit exactly at their upper
joint limit (0.0) in this pose.

## Bring-up / tear-down

`prime` (shared by the driver and the CLI, in `motorbridge_helpers`): open bus → clear latched motor errors
→ enable all → arm to POS_VEL (writing loop-gain registers) → gripper to FORCE_POS → close gripper at the
capped torque → stream a slow (0.5 rad/s) POS_VEL move to PRIME and poll convergence. `unprime` reverses:
close gripper, park at REST (the home pose, where the folded arm is safe to de-energize — this mirrors the
vendor's own shutdown), disable motors. The serial bridge stays open across unprime/prime cycles.

`halt` / `resume` (stale-command watchdog path): the DM firmware has no STOP state, so halt latches the
measured pose as the POS_VEL target (or zeros the VEL target) and the driver refuses writes until resume.
The firmware also has its own CAN-timeout protection (RID 9 / `set_can_timeout_ms`) as a deeper backstop we
don't currently configure.

## Streaming safety

`write_joint_positions` clamps the commanded target's advance to `joint_speed_limit_rad_s * dt` from the
previously commanded position (re-seeded from the measured pose after prime / resume / mode changes), the
same client-side rate limiter as the Lite6 driver. The firmware POS_VEL velocity limits (5.0 rad/s on
joints 1-3, 3.0 on 4-6, from the vendor config) bound the chase speed as a second layer.

## Teleop / leader arm (future work)

The leader is the StarArm102 (FashionStar UART servos, `motorbridge-smart-servo` package), read-only. The
LeRobot pattern for safe mirroring, for when we wire this up:

- Both arms calibrated to the same physical zero pose, so joint values map 1:1.
- Follower arm in soft MIT (kp 45/45/45/8/9/8, kd 12/12/12/1/1/1) — compliant, so a large mirrored error
  (e.g. near the folded REST pose) produces bounded torque and the arm yields instead of breaking its
  printed parts. NOT stiff position control.
- Per-command soft joint-limit clipping, plus an optional `max_relative_target` cap on how far any single
  command may jump from the present position.
- Gripper mirrored through the same FORCE_POS torque-capped path as everything else.

## Aegis

Sim: `aegis run kylos --config rebot_b601_dm` (the sim backend is manipulator-agnostic; gains in
`RebotB601DmModel.get_default_sim_pid_gains` are untuned initial values). Hardware: set `mode: hardware` in
`configs/aegis/rebot_b601_dm_ac.yaml`; the driver config block (`rebot_b601_dm_driver_config`) carries the
required `joint_speed_limit_rad_s` and `gripper_torque_ratio`.

## Not yet validated on hardware

Everything in this package was written against the vendor stack's validated sequences but has NOT itself
been run against the physical arm yet. First hardware session checklist: `probe`, `zero` at the home pose,
`stream --passive` while moving by hand, `gripper -w 0.05` against a soft object (verify the torque cap),
`send_jp` small wiggles, then `stream` (full prime/unprime cycle), then aegis hardware mode.
