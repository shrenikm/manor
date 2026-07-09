# manor.manipulators.rebot_b601_dm

Notes on the Seeed Studio reBot Arm B601 DM and how manor drives it. The arm is 6-DOF on Damiao CAN motors
(DM-J4340 on joints 1-3, DM-J4310 on joints 4-6) plus an actuated parallel gripper (a seventh DM-J4310).
All seven motors sit on one CAN bus behind a Damiao serial bridge (`/dev/ttyACM0`, 921600 baud), spoken to
via the `motorbridge` pip package.

These findings come from reading the vendor stack end to end, since getting the control wrong physically
breaks the arm (see below). All links below were confirmed live on 2026-07-08 — start here next time
instead of searching.

### Resources

Control stack (the definitive source for what commands are safe — read the LeRobot follower first):

- LeRobot follower (the authority on teleop / streaming control):
  [rebot_b601_follower.py](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/rebot_b601_follower/rebot_b601_follower.py),
  [config_rebot_b601_follower.py](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/rebot_b601_follower/config_rebot_b601_follower.py)
  — `control_mode` (mit / pos_vel), per-joint gains, `max_relative_target`, soft joint limits, gripper
  FORCE_POS ratio.
- LeRobot leader (read-only teleop source):
  [rebot_102_leader.py](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/rebot_102_leader/rebot_102_leader.py).
- LeRobot Damiao motor bus (a second, pure-Python reference for the CAN encoding / limits):
  [damiao.py](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/damiao/damiao.py).
- Seeed's own LeRobot integration repos:
  [lerobot-robot-seeed-b601](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601) (follower),
  [lerobot-teleoperator-seeed-b601](https://github.com/Seeed-Projects/lerobot-teleoperator-seeed-b601),
  [lerobot-teleoperator-rebot-arm-102](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102) (leader).
- [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) — low-level SDK (JointGroup
  architecture, mode sequences, gains).
- [reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) — ROS2 node (state
  machine, gripper services, gravity compensation) and the source of the URDF used in robot_models.
- [fashionstar-starai-arm-ros2](https://github.com/Seeed-Projects/fashionstar-starai-arm-ros2) — the
  FashionStar leader-servo side.

Motor SDK (the layer manor actually calls):

- [motorbridge](https://motorbridge.seeedstudio.com) — the pip package manor pins; `Controller` /
  `Motor`, the Damiao send_mit / send_pos_vel / send_force_pos primitives, register IDs.
- [Stackforce-Motor-SDK](https://github.com/Seeed-Projects/Stackforce-Motor-SDK) — the vendor SDK behind
  motorbridge.

Hardware, wiki, product overview:

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) — hardware / mechanical / URDF source repo
  (no control code; that lives in the repos above).
- Seeed wiki: [LeRobot getting started](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/),
  [ROS2 integration](https://wiki.seeedstudio.com/rebot_arm_b601_dm_ros2_integration/).
- [Hugging Face LeRobot docs — reBot B601](https://huggingface.co/docs/lerobot/main/en/rebot_b601).

## Motor control modes

The DM motors expose four firmware modes (motorbridge `Mode`):

- `MIT` — impedance command `(p_des, v_des, kp, kd, tau_ff)`; torque = kp·err + kd·verr + tau_ff, with
  NO integrator, saturating at the motor's max torque. Bounding the commanded error therefore bounds the
  torque. This is what the official LeRobot follower defaults to for the arm (`control_mode="mit"`, kp
  45/45/45/8/9/8, kd 12/12/12/1/1/1) and what our driver streams joint positions AND velocities with
  (velocities as pure damping commands, kp = 0). The vendor's stiffer endpos gains (kp 120 / kd 8 on the
  4340s, 18 / 2 on the 4310s) are used for our torque-bounded bring-up moves.
- `POS_VEL` — cascaded position/velocity loop inside the motor, gains in registers (RID 25/26 velocity
  loop, 27/28 position loop; vendor gains: 4340P `(0.0125, 0.004, 150.0, 0.5)`, 4310
  `(0.0008, 0.002, 50.0, 1.0)`). Each command carries a velocity limit — but the loops have INTEGRAL
  terms, so a blocked joint winds up to full motor torque. The vendor's ROS2/MoveIt stack uses this for
  collision-checked planned motion; opt-in via `arm_control_mode: pos_vel`, never near contact.
- `VEL` — firmware velocity mode; same integrator-windup caveat. Used only by the CLI's send_jv
  experiment and the pos_vel driver mode.
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

Gripper geometry (established on hardware 2026-07-06): the FORCE_POS command frame and the feedback
frame DIFFER. Commands are rotor-side (10:1, sign-inverted vs output): 0 = closed, −5 = fully open — an
over-command past the physical stop (~−3.13 rotor rad) that the torque cap stalls at, exactly how the
vendor uses it. Feedback is output-side: 0 = closed, +0.3134 rad at the open stop (positive opens). Width
0–0.143 m maps linearly onto each frame via `gripper_width_to_motor_rad` (commands) and
`gripper_motor_rad_to_width` (feedback); never echo a feedback position back as a command.

## Zeroing / calibration

The motors have no persistent calibration; the vendor tooling re-zeroes against whatever pose the arm holds
at setup. The canonical zero is the vendor home pose: shoulder and forearm horizontal, gripper pointing
forward and fully closed (identical to the URDF q = 0). Use `rebot_b601_dm zero` with the arm physically
held there, then verify with `rebot_b601_dm stream --passive`. Joints 2 and 3 sit exactly at their upper
joint limit (0.0) in this pose.

## Bring-up / tear-down

`prime` (shared by the driver and the CLI, in `motorbridge_utils`): open bus → clear latched motor errors
→ enable all → gripper to FORCE_POS and closed at the capped torque → torque-bounded interpolated MIT move
to PRIME (target stepped at 0.5 rad/s, commanded position clamped within 0.08 rad of measured, vendor endpos
gains — a blocked move pushes with ~10 N·m max at joints 1-3 until the timeout aborts, instead of a POS_VEL
integrator winding to 27). The driver then switches to POS_VEL only if that mode is configured. `unprime`
reverses: close gripper, park at REST via the same bounded move (the home pose, where the folded arm is safe
to de-energize — mirrors the vendor's own shutdown), disable motors. The serial bridge stays open across
unprime/prime cycles.

`halt` / `resume` (stale-command watchdog path): the DM firmware has no STOP state, so halt latches the
measured pose as a soft MIT hold (or the POS_VEL target / zeroed VEL target in those modes) and the driver
refuses writes until resume.
The firmware also has its own CAN-timeout protection (RID 9 / `set_can_timeout_ms`) as a deeper backstop we
don't currently configure.

## Streaming safety — THE ARM IS ALSO PARTLY 3D PRINTED

Every streamed position command passes through two clamps:

1. Rate limiter: advance capped at `joint_speed_limit_rad_s * dt` from the previously commanded position
   (re-seeded from the measured pose after prime / resume / mode changes) — bounds motion speed, same as
   the Lite6 driver.
2. Command-error clamp (`max_command_error_rad`, required config): the command never leads the freshly
   MEASURED position by more than this. In the MIT default this is the arm's torque bound — per joint,
   torque stays below ~kp × clamp (≈6.8 of 27 N·m at joints 1-3, ≈1.3 of 7 at 4-6 with 0.15 rad), so a
   blocked arm (contact, self-collision at the folded zero pose, a bad IK target) yields instead of
   breaking. This mirrors the LeRobot follower's `max_relative_target` mechanism.

Velocity commands in MIT mode are pure damping (torque = kd × velocity error), bounded by construction.
In `pos_vel` mode neither clamp bounds torque (firmware integrators) — planned, collision-checked motion
only.

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
