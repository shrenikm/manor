# Plan 1 for Manor

I want to do a full redesign of how the current systems are set up for manipulation.
The end goal is still the same -- to be able to run the robot both in simulation and on the hardware through the same code.

The initial idea was to have a "pliant" (Drake plant but pliant as it's flexible for hardware and sim) but it's currently mainly a monolithic system.
We currently also don't take any sensor inputs, etc.

I want the system to be split into multiple modules that both publish and subscribe to messages. Why?

1. I want to use this project for both classical motion/control and also more modern learning based policies (VLAs, etc)
2. To run ML policies, I need to run it on a remote server (with GPU) so this needs to be distributed system
3. I currently don't have any inputs (images, depth, etc) so I need to integrate this into the entire system as well

The plan now is that we are going to have individual systems:

- Helios: The system that observes the environment and publishes messages for sensor output (RGB, RGBD, etc)
- Soma: The system that publishes the full proprioception state. Takes in the joint and gripper state and outputs the joint, gripper and EEF state (EEF after forward kinematics)
- Metis: The system that takes in the observations and proprioception and outputs desired action (some kind of policy). Can be classical policy like a classical motion planner or a learning based policy like a VLA. The action can be joint and gripper positions/velocities or EEF positions/velocities. Or they can be trajectories.
- Kyber: The system that is a controller that takes in an action and outputs commands for a robot. The output is either joint positions or velocities along with the control for the gipper
- Talos: System that is the hardware interface for specific robots (or for simulation). Takes in the commands and executes them on hardware (or simulation)


The first part of this plan is going to be non destructive -- we're just going to be creating a bunch of python classes for messages for the systems described above.

Each message class will be a frozen python attrs class that will extend three interfaces: ISerializable and ILcmMessage

These classes define the following abstract methods:

ISerializable: serialize(), deserialize()
ILcmMessage: to_lcm_message(), from_lcm_message()

Most of these are self explanatory.

Each class must store a Version number as a class attribute (use semantic versioning)

For serialization use capnproto. To handle versioning cleanly, we have a capnproto schema class for each version of the message (V1, V2 etc)
Then during deserialization, we can read the version number in the bytes and then create the python class.

We might have to store MessageV1, MessageV2 etc and then a VersionedMessage with the individual union messages that keeps growing for the versioning. But I'll leave the implementation to you.
Just make sure that whatever we do here is clean and scalable

So serialize will go from python message -> capnp -> bytes
Deserialize will go from bytes -> capnp -> python message

Lcm stuff is simple - Each python message class will have a corresponding LCM message class. We should be able to convert between the two easily.
Note: There is no versioning here. The lcm messages will always reflect the latest version of the message. Even if we store old serialized data in telemetry, we can deserialize to the latest version of the message and then convert to the latest LCM message for pub/sub.

So here's the structure:

- Each of these python messages will have a separate file under definitions/
- Each of these message classes will have a separate file for the capnp schema
- Each of these mesage classes will have a separate lcm file

I'll leave you to decide where to place the lcm and capnp files. Just makes sure that they're installed through the pyproject so they can be used.
We probably also need compilation code for lcm (not sure about capnp). Feel free to add something in <root>/scripts/ to compile the lcm files into python. It would be nice if the pyproject could
run this script during installation as well.

Here are messages:

TimestampHeader:
  monotonic_ns: int
  system_ns: int

RGBImageData:
  header: TimestampHeader
  // Figure out what else need here. I think height, width, encoding, data (bytes), etc. We must be able to store both compressed and uncompressed images.

RGBDImageData:
  header: TimestampHeader
  // Figure out

JointPositions:
  header: TimestampHeader
  positions: np.ndarray

JointPositionsTrajectory:
  header: TimestampHeader
  times: np.ndarray // seconds (SI units so not denoting with times_s)
  joint_positions_array: np.ndarray // 2D array -- number of time steps x number of joints

JointVelocities:
  header: TimestampHeader
  velocities: np.ndarray

JointVelocitiesTrajectory:
  header: TimestampHeader
  times: np.ndarray
  joint_velocities_array: np.ndarray // 2D array -- number of time steps x number of joints

JointState:
  header: TimestampHeader
  joint_positions: JointPositions
  joint_velocities: JointVelocities

JointStateTrajectory:
  header: TimestampHeader
  times: np.ndarray
  joint_positions_array: np.ndarray // 2D array -- number of time steps x number of joints
  joint_velocities_array: np.ndarray // 2D array -- number of time steps x number of joints

// EEF Can be a gripper, maybe a hand with fingers. etc.
// EEF position/velocities are the generalized coordinates
EEFPositions:
  header: TimestampHeader
  positions: np.ndarray

EEFPositionsTrajectory:
  header: TimestampHeader
  times: np.ndarray
  eef_positions_array: np.ndarray // 2D array -- number of time steps x number of EEF generalized coordinates

EEFVelocities:
  header: TimestampHeader
  velocities: np.ndarray

EEFVelocitiesTrajectory:
  header: TimestampHeader
  times: np.ndarray
  eef_velocities_array: np.ndarray // 2D array -- number of time steps x number of EEF generalized coordinates

EEFState:
  header: TimestampHeader
  eef_positions: EEFPositions
  eef_velocities: EEFVelocities

EEFStateTrajectory:
  header: TimestampHeader
  times: np.ndarray
  eef_positions_array: np.ndarray // 2D array -- number of time steps x number of EEF generalized coordinates
  eef_velocities_array: np.ndarray // 2D array -- number of time steps x number of EEF generalized coordinates

// This is for the pose of the actual EEF point (center of the gripper, etc). Not the individual positions/joints of the EEF
EEFPose:
  header: TimestampHeader
  translation: np.ndarray
  orientation: np.ndarray // quaternion

EEFPoseTrajectory:
  header: TimestampHeader
  times: np.ndarray
  translations_array: np.ndarray // 2D array -- number of time steps x 3 (x, y, z)
  orientations_array: np.ndarray // 2D array -- number of time steps x 4 (quaternions)

EEFTwist:
  header: TimestampHeader
  linear: np.ndarray
  angular: np.ndarray

EEFTwistTrajectory:
  header: TimestampHeader
  times: np.ndarray
  linear_array: np.ndarray // 2D array -- number of time steps x 3 (x, y, z)
  angular_array: np.ndarray // 2D array -- number of time steps x 3 (roll, pitch, yaw)

// Full proprioception state
Proprioception:
  header: TimestampHeader
  joint_state: JointState
  // eef states may be optional as we might not have an EEF for a robot, or we might choose to not do forward kinematics and publish the EEF pose, etc.
  eef_state: EEFState | None
  eef_pose: EEFPose | None
  eef_twist: EEFTwist | None


// All observations that are input to the policy
// Can be empty with just the header as we might wanna run hardcoded policies to test control
Observation:
  header: TimestampHeader
  proprioception_state: ProprioceptionState | None
  rgb_image: RGBImageData | None
  rgbd_image: RGBDImageData | None
  // .. More to be added as we have more sensors.

Action:
  header: TimestampHeader
  // Only one of these can be set (need to add an attrs validator for this)
  joint_positions: JointPositions | None
  joint_positions_trajectory: JointPositionsTrajectory | None
  joint_velocities: JointVelocities | None
  joint_velocities_trajectory: JointVelocitiesTrajectory | None
  eef_pose: EEFPose | None
  eef_pose_trajectory: EEFPoseTrajectory | None
  eef_twist: EEFTwist | None
  eef_twist_trajectory: EEFTwistTrajectory | None

Command:
  header: TimestampHeader
  // Can only be one of these. Most common are joint pos/vel as we probably want to do our own IK instead of controlling in EEF space and using
  // an external robot API to do IK, but we allow it here just in case.
  joint_positions: JointPositions | None
  joint_velocities: JointVelocities | None
  eef_pose: EEFPose | None
  eef_twist: EEFTwist | None

Let's start by implementing these classes

Let's leave tests out for now. Once I have confirmed that the messages look good, we can write a lot of thorough tests.

# Implemented

## Final state

- **24 message classes** under `src/manor/common/definitions/`, one file per message. Every class is `@attr.frozen` and inherits from `DefinitionBase`, which composes two interfaces:
  - `ISerializable` — `serialize() -> bytes` / `deserialize(bytes) -> Self` via Cap'n Proto.
  - `ILcmMessage` — `to_lcm_message()` / `from_lcm_message(msg)` for the generated `lcmt_*` classes.
- The set covers everything in the plan plus a standalone `DepthImageData` (so `RGBDImageData` composes RGB + depth instead of duplicating depth fields):
  `TimestampHeader`, `RGBImageData`, `DepthImageData`, `RGBDImageData`,
  `JointPositions(/Trajectory)`, `JointVelocities(/Trajectory)`, `JointState(/Trajectory)`,
  `EEFPositions(/Trajectory)`, `EEFVelocities(/Trajectory)`, `EEFState(/Trajectory)`,
  `EEFPose(/Trajectory)`, `EEFTwist(/Trajectory)`,
  `Proprioception`, `Observation`, `Action`, `Command`.
- **Discriminated-union messages** (`Action`, `Command`) enforce "exactly one variant set" in `__attrs_post_init__`; they persist their active variant via a tag (`variant: int8`) on LCM and via a Cap'n Proto union arm on capnp.
- **Optional sub-messages** in `Proprioception` / `Observation` use Cap'n Proto union groups (`some` / `none`) and a `has_X` flag on LCM.

## Versioning

- Each message has a top-level Cap'n Proto wrapper `VersionedX` that's a union over `v1`, `v2`, ... arms.
- The Python class declares `CURRENT_CAPNP_VERSION: ClassVar[str]` (e.g. `"v1"`), implements `to_capnp_current(builder)`, and per-version `from_capnp_vN(reader)` classmethods.
- `serialize()` always writes the current version arm. `deserialize()` reads whichever arm is active and dispatches to the matching `from_capnp_vN`. Adding a new version is purely additive: new `.capnp` arm + bumped `CURRENT_CAPNP_VERSION` + new `from_capnp_vN`, leave existing converters intact.
- **No LCM versioning** per the plan: `lcmt_*` always reflects the latest schema. Old serialized capnp bytes still round-trip cleanly via `from_capnp → to_lcm_message`.

## Schemas + build

- `definitions/schemas/capnp/` — 25 `.capnp` files (one per message + a shared `common.capnp`).
- `definitions/schemas/lcm/` — 24 `.lcm` files, all declaring `package manor.common.definitions.lcmtypes;`.
- `definitions/lcmtypes/` — Python LCM bindings, generated from the `.lcm` files.
- `scripts/compile_messages.py` runs `lcm-gen --python --ppath src/` to regenerate the bindings in-place under the package tree and validates each capnp schema with a fresh `SchemaParser`.
- `scripts/build_hook.py` is a hatchling custom build hook that invokes `compile_messages.py` during `uv pip install --no-cache-dir -e .`, so editable installs always have fresh bindings.

## Beyond the plan

- **Tests**: ~70 pytest tests in `definitions/tests/`, including round-trip coverage (capnp serialize → deserialize, LCM to/from) for every message. The plan called tests out as "later" — they were added.
- **Custom exception hierarchy**: `ManorError → DefinitionError → InvalidDefinitionError`, plus `SerializationError`. Used by variant-validation failures and unknown capnp arms / LCM tags.
- **Capnp utilities** in `definitions/utils/capnp_utils.py` (e.g. `ndarray_to_float64_array`, `float64_array_to_ndarray`, `load_versioned_schema`).
- **Image encoding enums** in `definitions/utils/enums.py` (`ImageEncoding`, `DepthEncoding`).
- **Test factories** in `definitions/tests/factories.py` for building well-formed instances of any message during testing.

## Out of scope / pre-existing

- `control_definitions.py` and `state_definitions.py` are pre-existing legacy modules from before the redesign and are not part of this plan; they will be removed or rewritten as the rest of the codebase migrates off them.

