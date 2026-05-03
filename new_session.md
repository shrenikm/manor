Hi! We've been working on this project together for a long time. To get up to
speed, please read the README under `src/manor/common/aegis/README.md` for the
aegis stack, and `src/manor/manipulators/lite6/README.md` for the empirical
notes on the Ufactory Lite6 / xarm-python-sdk hardware bring-up.

Where we are right now (as of 2026-05-01):

- We've been doing a deep refactor of the project. aegis (the new stack) is in
  good shape; the old code coexists with it in the repo and will get nuked once
  the new path is fully working end-to-end.
- The active branch is `aegis`.

Things that have changed since this file was first written, so the aegis README
is the canonical reference but a few callouts:

- **Talos FK is no longer stubbed.** Talos owns its own FK-only
  `MultibodyPlant`, syncs the plant context against the backend's
  proprioception each tick, and produces a real `CartesianPose` (via
  `CalcRelativeTransform`) and `CartesianTwist` (via
  `CalcJacobianSpatialVelocity`). The `CartesianState` published inside
  `Proprioception` is correct, not a placeholder.
- **The gripper is independently commanded in sim.** `Gaia` now exposes
  `apply_ee_position_command` / `apply_ee_velocity_command`, and
  `SimManipulatorBackend.send_joint_ee_command` routes the `ee_command`
  side of `JointEECommand` through them. Open/close cycles work in sim.
- **A diff-IK controller landed.** `IKPassthroughController`
  (`KyberControllerType.IK_PASSTHROUGH`) handles Cartesian-shaped actions:
  `cartesian_pose` goes through Drake's `InverseKinematics`, `cartesian_twist`
  goes through `DoDifferentialInverseKinematics`. The EE block is locked
  (position) or pinned to zero velocity (twist) so all motion is allocated
  to the arm DOFs; joint commands and EE commands fall through unchanged.
- **New policies shipped.** Beyond the early bring-up policies, there's now
  `circle_ee_velocity` (constant-speed circle traced in the world xy-plane
  via cartesian twists), `constant_cartesian_pose` (fixed pose target paired
  with the IK passthrough controller), and `gripper_open_close` (cycles the
  EE between its model-defined upper/lower position limits).
- **Config layout is split into base + sub-YAMLs.** The bundled config is
  `configs/aegis/lite6_ac.yaml` (one base per manipulator type); it names a
  policy and controller via `metis_config.policy_type` and
  `kyber_config.controller_type`. The matching bodies live in
  `configs/aegis/policies/<type>_ac.yaml` and
  `configs/aegis/controllers/<type>_ac.yaml`. The composer
  (`compose_aegis_yaml_dict` in `aegis.py`) inlines them; aegis refuses to
  run if a sub-YAML is missing or malformed.

Things still stubbed / known-limited:

- **Helios sim backend still returns empty RGB-D frames.**
  `Gaia.render_rgb` / `render_depth` are structural stubs (return the empty
  template). The hardware Helios backend is also a TODO stub. Channels
  publish at the configured rates but payloads are placeholders.
- **The hardware Helios backend isn't wired up yet** — `start` / `stop` /
  `read_rgb` / `read_depth` are TODOs. The xarm `Lite6Driver`-side
  manipulator backend on hardware works.
- **gylos runs slightly below realtime** — kyber at 500 Hz + helios at 30 Hz
  + the gaia advancer + continuous-time integration adds up. Switching to
  discrete `gaia_config.time_step` (e.g. `0.001`) speeds it up at the cost
  of integration accuracy.

What "old code" still hanging around looks like:

- The pre-aegis Lite6 pliant (`Lite6Pliant`, `Lite6PliantMultiplexer`,
  `Lite6PliantChoreographer`, etc.) and the per-block legacy pipelines in
  `src/manor/` outside `src/manor/common/aegis/`. The aegis stack borrows
  the priority logic and some constants from there but builds a fresh
  Drake graph; once aegis is feature-complete we'll delete the old paths.
- See feedback memory `feedback_drake_usage.md` — when in doubt about Drake
  patterns, prefer the Drake docs / API, not the legacy manor code.

Quick orientation by directory:

- `src/manor/common/aegis/` — the new runtime (Metis / Kyber / Talos /
  Helios / Gaia / GaiaAdvancer + LCM adapters + CLI + per-block runners).
- `src/manor/common/aegis/metis/policies/` — policy implementations + the
  `MetisPolicyManager` factory.
- `src/manor/common/aegis/kyber/controllers/` — controller implementations
  + the `KyberControllerManager` factory.
- `src/manor/manipulators/lite6/` — Lite6 model, driver, hardware-bringup
  CLI (`lite6_cli.py`) used to characterise the xarm SDK.
- `configs/aegis/` — the new split config layout (base + policies/ +
  controllers/).
- `models/` — URDFs / SDFs for environment objects (tables, etc).
- `robot_models/` — git submodule with robot URDFs (Lite6 etc).

Feel free to read whatever else you need to get a feel for the project.
