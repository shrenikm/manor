## Plan for action/command definitions

I realize that there are some issues:

- The names EEFPose (for pose of the tip of the EEF) and EEFPositions for positions of the actual EEF are confusing
- Command actually contains the wrong thing - EEFPose/Twist instead of EEFPositions/Velocities
- We should call everything "EE" instead of "EEF"

So here's the plan, we segment these commands into 3 categories: Joint, Cartesian and EE where cartesian is the pose/velocity of the end-effector (task/operation space control)

So the main idea is that Metis outputs Actions (which can contain higher level things like cartesian trajectories) but Talos only takes in "commands" which only has joint level control (for both the main arm and the EE). So Kyber is tasked with converting the actions into commands which may require IK to go from cartesian to join and maybe some kind of tracking algorithm or trajectory optimization to go from trajectories to instantaneous commands in positions/velocities.

So basically the current EEFPose, EEFTwist will be renamed to CartesianPose and CartesianTwist, same with the trajectories.
Also EEFPositions and EEFVelocities will be renamed to EEPositions and EEVelocities.

Definitions:

All of these in different files.

// only one can be given
JointCommand:
  JointPositions
  JointVelocities

// only one can be given
JointTrajectoryCommand:
  JointPositionsTrajectory
  JointVelocitiesTrajectory

// only one can be given
CartesianCommand:
  CartesianPose
  CartesianVelocity

// only one can be given
CartesianTrajectoryCommand:
  CartesianPoseTrajectory
  CartesianTwistTrajectory

// only one can be given
EECommand:
  EEPositions
  EEVelocities

// only one can be given
EETrajectoryCommand:
  EEPositionsTrajectory
  EEVelocitiesTrajectory

The command class now becomes:

// Joint command must be given and EECommand is optional
Command:
  JointCommand
  EECommand

Action now becomes:

// Group1: Joint and cartesian commands. Exaclty 1 of these 4 must be set (others None/optional)
// Group2: EE commands. Both optional but a maximum of 1 can be set.
Action:
  JointCommand
  JointTrajectoryCommand
  CartesianCommand
  CartesianTrajectoryCommand
  EECommand
  EETrajectoryCommand


Please make sure:

- Follow the existing design patters while writing these
- Make sure each of these classes still follow the capnrpto and lcm stuff. Need to follow the same capnp versioning convention we have been using, etc.
- Please write thorough tests for everything. Make sure we have tests covering all combinations of the instances for things like commands and actions (where only 1 can be set etc. So we test that none being set raises an error, more than 1 raises and error and we verify everything (serialization, etc) for all combinations)
- Please also replace all "EEF" that refer to end effector to "EE". Also any "eef_var" to "ee_var" and so on. Update this in the entire code base, documentaation etc.
- Please also note any of these change sin existing documentaiton or other code to make updates.

---

## Implementation addendum (locked in 2026-04-29)

### Resolved decisions

- **CartesianCommand variants:** CartesianPose / CartesianTwist (not "CartesianVelocity").
- **EEState rename:** EEFState -> EEState. Fields: header, ee_positions, ee_velocities. All required, none optional.
- **CartesianState (new, for symmetry):** Fields: header, cartesian_pose, cartesian_twist. All required.
- **Proprioception shape:** header (required), joint_state (required), cartesian_state: CartesianState | None (optional), ee_state: EEState | None (optional). The wholeshow sub-state may be None; if present, every inner field is non-None.
- **Capnp versioning:** start every renamed/new schema fresh at v1. No on-disk messages exist yet; this is still development code.
- **Container naming:** keep the verbose "Command" suffix (JointCommand / JointTrajectoryCommand / CartesianCommand / CartesianTrajectoryCommand / EECommand / EETrajectoryCommand). Top-level Command contains a JointCommand by composition.
- **Validators:** every attrs class with a one-of / at-most-one-of / required-required-optional invariant must enforce it in __attrs_post_init__. Failures raise InvalidDefinitionError. Audit existing Command + Action validators while we're at it; they already exist and we keep that idiom.

### File rename map (phase 1)

Pure renames, no semantic change. Each touch updates the .py, the .capnp schema, the LCM .lcm schema, and every importer:

- eef_pose.py            -> cartesian_pose.py
- eef_pose_trajectory.py -> cartesian_pose_trajectory.py
- eef_twist.py           -> cartesian_twist.py
- eef_twist_trajectory.py -> cartesian_twist_trajectory.py
- eef_positions.py            -> ee_positions.py
- eef_positions_trajectory.py -> ee_positions_trajectory.py
- eef_velocities.py            -> ee_velocities.py
- eef_velocities_trajectory.py -> ee_velocities_trajectory.py
- eef_state.py            -> ee_state.py        (already holds only positions+velocities; pose/twist live in Proprioception today and move into the new CartesianState in phase 2)
- eef_state_trajectory.py -> ee_state_trajectory.py

Class names: EEFPose -> CartesianPose, EEFTwist -> CartesianTwist, EEFPositions -> EEPositions, EEFVelocities -> EEVelocities, EEFState -> EEState, etc. Module-level capnp/lcm types renamed in lockstep (VersionedEEFPose -> VersionedCartesianPose, lcmt_eef_pose -> lcmt_cartesian_pose, etc.).

Attribute names: eef_pose -> cartesian_pose, eef_twist -> cartesian_twist, eef_positions -> ee_positions, eef_velocities -> ee_velocities, eef_state -> ee_state.

### New files (phase 2-3)

- cartesian_state.py + cartesian_state.capnp + lcmt_cartesian_state.lcm
- joint_command.py + joint_command.capnp + lcmt_joint_command.lcm
- joint_trajectory_command.py + ...
- cartesian_command.py + ...
- cartesian_trajectory_command.py + ...
- ee_command.py + ...
- ee_trajectory_command.py + ...

Each container .py defines a frozen attrs class with header + N variant fields (each Optional, default None), a CURRENT_CAPNP_VERSION = "v1" ClassVar, capnp_arm dispatch, lcm variant-int dispatch, and a __attrs_post_init__ enforcing exactly one variant set. Mirrors today's Action / Command idiom.

Capnp pattern: top-level VersionedX struct wraps a v1 union over the variant arms, same as the existing action.capnp schema.

LCM pattern: top-level lcmt_x has header + int8 variant tag + one nested struct per arm; only the active arm carries meaningful data. Matches lcmt_action / lcmt_command.

### Rewritten classes (phase 4-6)

- Command: header, joint_command: JointCommand (required), ee_command: EECommand | None = None. Validator enforces joint_command non-None (attrs handles required-no-default already) and ee_command consistency. No one-of at this level.
- Action: header, joint_command, joint_trajectory_command, cartesian_command, cartesian_trajectory_command (group 1: exactly one), ee_command, ee_trajectory_command (group 2: at most one). Validator enforces both constraints; tests cover all 4 group-1 singletons x {None, ee_command, ee_trajectory_command} = 12 valid permutations + invalid cases (none in g1, two in g1, two in g2).
- Proprioception: header, joint_state, cartesian_state | None, ee_state | None. No one-of; just mark joint_state required and the other two optional.

### Phases (ordered, each leaves test suite green)

1. Pure renames: EEF -> EE / EEF -> Cartesian on bare primitives + their schemas + every reference. Sweep code, configs, README, comments, docstrings.
2. Add CartesianState (new attrs class + schemas + tests).
3. Add the 6 container defs + schemas + tests (validators included in every one).
4. Rewrite Command around (joint_command, ee_command). Update both Talos backends, all Kyber controllers, all default-construction sites.
5. Rewrite Action around the 6 container fields + 2-group validator. Update all Metis policies, test_metis fixtures, _RecordingPolicy in tests.
6. Rewrite Proprioception (joint_state required, cartesian_state / ee_state optional). Update Talos, Gaia, _DesiredStateSource if it touches the sub-states. Update aegis README to match.
7. Final pass: any straggling EEF / eef_ in code / docs / xarm notes / configs / mermaid diagrams.

### Test coverage policy

- Every container: parametrized happy-path tests covering each singleton variant; capnp round-trip + lcm round-trip per variant; equality across construction-via-each-arm; validator tests for none-set and multi-set.
- Action: cross-product tests for the (group1 x group2) matrix, capnp + lcm round-trip on representatives, invalidator tests for both groups.
- Command: required joint_command + optional ee_command tests, capnp + lcm round-trip per (joint variant x ee state).
- Proprioception: round-trip with all 4 combinations of (cartesian_state present/None, ee_state present/None).
- Validator-specific tests live alongside the type: every InvalidDefinitionError path is exercised.

### Notes on existing code paths to update

- aegis README example YAML and Caveats section (Cartesian / EE rename, no inline EEF text left).
- xarm_api.md (Lite6 README) -- mentions of "EEF state" / "eef_pose" if any. Verify with a final grep.
- aegis tests, kyber tests, metis tests, talos tests -- every fixture builds Action / Command / Proprioception by name.
- Gaia.apply_joint_*_command callers -- they touch Command.joint_positions / .joint_velocities directly today; that becomes Command.joint_command.positions etc.

