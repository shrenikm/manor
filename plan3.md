# Plan 3 for Manor

Continuing with the full redesign of the project, the next step is to write the new interfaces for the manipulator itself.
Note that I have renamed manor/manipulators/lite6 to manor/manipulators/deprecated_lite6 to make room for the upgrade. I dislike the old way of doing things and want a fresh refactor/redesign.

The object is to be flexible enough to support different manipulators and different model types of the same manipulator.
Currenlty there is only one manipulator --  the Ufactory Lite6 robot 
This has a bunch of different models/trims (with vaccum gripper, without any grippers, with parallel grippers in different orientations, etc)

Currently in Lite6ModelType there are 6 different models, but I want to reduce them and only use the necessary ones.
I wnat to get rid of the vacuum only models and the unactuated models. So we're left with 3: vacuum gripper, parallel gripper in the two configurations (normal and reverse)
Let's keep all of the existing urdfs from the other robot_models project and just map them differently to the model types (only the actuated ones will be used in Manor)

First, we need two enums:

class ManipulatorType(StrEnum):
  LITE6 = "lite6"

This is the base/top level enum for manipulator type. We also have a manipulator model type for the individual models/trims of the manpulator:

class ManipulatorModelType(StrEnum):
  
  @abstractmethod
  def get_manipulator_type(self) -> ManipulatorType:
    ...

class Lite6ModelType(ManipulatorModelType):
  ...

This will be the same enum as the existing deprecated Lite6ModelType but without the vacuum only models (so 6 types in total)

Next we need two interfaces, one of the model to get the characteristics of the manipulator (for sim, etc) and one for the hardware interface to actually move a physical robot.

Starting with the model:

class IManipulatorModel(ABC):

  ...

This class will have functionality for the following (it loosely follows the existing utils functions in lite6_model_utils.py)
- Get the manipulator type
- Get num dof (just the main joints, so independent of EEF)
- Get the path of the description file (urdf, but I also want to maybe support other formats in the future, so let's keep it generic and call it description file)
- Get the names of some of the important frames (base, end effector, etc)
- Get the number of positions, states, etc. I think these are used for multibody state vector mapping. Please confirm that it is used for that and if it is useful, add these functions.
- there are methods for filling in the state vectors, etc in the current deprecated functions but I'm not sure if these are necessary. Also I want to switch to using the definition classes, so let's skip this for now.

^ All these above functions need to be abstract.
Also please don't port any "gripper" stuff from the deprecated stuff. I don't want gripperstatus enum etc for now. Also we're switching to calling it end-effector to support non gripper tools as well.

class Lite6Model(IManipulatorModel):
  model_type: IManipulatorModelType

We can crate dict mappings from the different model types to the functions required for the interface implementation (roughly following the existing deprecated utils)

Note that the goal is that we can use this model instance as the only thing required to work with Drake to set up sim and also do FK/IK.

class IManipulatorDriver(ABC):

  @abstractmethod
  def prime() -> None:
     // Prime the arm for use after boot up or after a reset.

  @abstractmethod
  def unprime() -> None:
    // Unprime the arm after use before shutdown or reset.

  @abstractmethod
  def read_joint_positions() -> JointPositions:
    ...

  @abstractmethod
  def read_eef_positions() -> EEFPositions | None:
    ...

  @abstractmethod
  def read_joint_velocities() -> JointVelocities:
    ...

  @abstractmethod
  def read_eef_velocities() -> EEFVelocities | None:
    ...

  @abstractmethod
  def write_joint_positions(joint_positions) -> None:
    ...

  @abstractmethod
  def write_eef_positions(eef_positions) -> None:
    ...

  @abstractmethod
  def write_joint_velocities(joint_velocities) -> None:
    ...

  @abstractmethod
  def write_eef_velocities(eef_velocities) -> None:
    ...


And then we have the Lite6Driver which implements the IManipulatorDriver interface and has the actual code for communicating with the robot (either through serial or through the network, etc)
Please figure out:
1. If the xarm API has methods to do these things (reading/writing joint/eef positions/velocities). I have some old code in the repo that you can use for reference, but it was also using a very old API version so you might wanna look up the latest API docs to see if there are any changes.
2. Figure out how (and if possible) to wrap the API calls into the above interface.

# Implemented

## Final state

The new structure lives at `src/manor/manipulators/`; the old code stays untouched under `src/manor/manipulators/deprecated_lite6/` and will be removed once the rest of the project migrates off it.

### Top-level types

- `manipulator_type.py` — `ManipulatorType(StrEnum)`, currently with one member: `LITE6`.
- `manipulator_variant.py` — `IManipulatorVariant(StrEnum)`, an empty parent enum that declares an abstract `get_manipulator_type()` for subclass enums. Plus:
  - `register_manipulator_variant(manipulator_type)` — class decorator that registers a variant enum class against a `ManipulatorType`. Uses a bound `TypeVar` (`_VariantT = TypeVar("_VariantT", bound=IManipulatorVariant)`) so pyright preserves the concrete class type after decoration (otherwise `Lite6Variant.VACUUM_GRIPPER` member access errored against `type[IManipulatorVariant]`).
  - `get_variant_class(manipulator_type)` — registry lookup; raises `UnknownManipulatorTypeError` if the corresponding manipulator package was never imported.
  - `get_registered_manipulator_types()` — for cross-manipulator iteration.
- `manipulator_model.py` — `IManipulatorModel(ABC)` with: `get_manipulator_type`, `get_variant`, `get_num_dof`, `get_description_filepath`, `get_base_frame_name`, `get_eef_tip_frame_name`, `get_num_positions`, `get_num_velocities`, `get_num_states`. Position / velocity / state counts map directly to Drake's `MultibodyPlant.num_positions()` / `.num_velocities()` for state-vector slicing.
- `manipulator_driver.py` — `IManipulatorDriver(ABC)` with `prime` / `unprime` plus the eight read/write methods (joint + EEF positions / velocities). EEF read methods are typed `-> ... | None` so manipulators without continuous-actuation EEFs can return `None`.

### Lite6 implementation

Under `src/manor/manipulators/lite6/`:

- `variant.py` — `Lite6Variant`, decorated with `@register_manipulator_variant(ManipulatorType.LITE6)`. Three trims: `VACUUM_GRIPPER`, `PARALLEL_GRIPPER_NORMAL`, `PARALLEL_GRIPPER_REVERSE`. Vacuum-only and unactuated parallel-gripper trims from the deprecated codebase were intentionally dropped.
- `model.py` — `Lite6Model(IManipulatorModel)`, frozen attrs class with `variant: Lite6Variant`. Module-level constants: `LITE6_ARM_DOF = 6`, `LITE6_PARALLEL_GRIPPER_DOF = 2`. Two dicts map each variant to its URDF filename and position count. Frame names (`link_base`, `link_eef_tip`) are constant across all three variants. Description files come from `robot_models/lite6_description/drake_urdf/robot_with_gripper/`. `get_manipulator_type()` delegates to `self.variant.get_manipulator_type()` rather than hardcoding a return.
- `driver.py` — `Lite6Driver(IManipulatorDriver)`. Wraps `xarm.wrapper.XArmAPI`. Uses lazy import (`try: from xarm.wrapper import XArmAPI; except ImportError: XArmAPI = None`) so the class can be instantiated without the SDK installed; `prime()` raises `Lite6DriverError` with an install hint if `XArmAPI is None`.

### Exceptions

Added to `src/manor/common/exceptions.py`:
- `ManipulatorError` (base, extends `ManorError`)
- `UnknownManipulatorTypeError`, `VariantAlreadyRegisteredError`
- `ManipulatorDriverError`, `Lite6DriverError`

## Deviations from the plan

- **Naming**: `ManipulatorModelType` → `IManipulatorVariant` (and `Lite6ModelType` → `Lite6Variant`); `Lite6Model.model_type` → `Lite6Model.variant`. The "model type" name conflicted with `IManipulatorModel`; "variant" matches the trim/configuration vocabulary.
- `get_description_path` → `get_description_filepath` for clarity.
- `register_variant_for` → `register_manipulator_variant` (more searchable).
- `Lite6Variant.PARALLEL_GRIPPER` was renamed to `PARALLEL_GRIPPER_NORMAL` so it parallels `PARALLEL_GRIPPER_REVERSE`.
- `Lite6Model.get_manipulator_type()` delegates to the variant rather than hardcoding `ManipulatorType.LITE6` (the variant already encodes this).
- Variant registry implemented via decorator (`@register_manipulator_variant(ManipulatorType.LITE6)`) rather than imperative `register_variant_class(...)` calls — declarative, registers at class-definition time.
- No top-level `ManipulatorModelType` alias / Union added: the abstract base `IManipulatorVariant` is itself a sufficient supertype for cross-manipulator typing.

## Beyond the plan

- **Tests**: ~46 pytest tests under `manipulators/tests/` and `manipulators/lite6/tests/`. Coverage:
  - `test_manipulator_variant.py` — registry round-trip, unknown-type lookup raises, double-registration raises.
  - `test_variant.py` — `Lite6Variant` is an `IManipulatorVariant` subclass; `get_manipulator_type()` returns LITE6 for every member; string values are stable.
  - `test_model.py` — Lite6Model is an IManipulatorModel; arm DOF constant; vacuum has no extra DOFs; parallel grippers add two DOFs; description file actually exists on disk; frame names match.
  - `test_driver.py` — patches `XArmAPI` with a `MagicMock`; covers prime/unprime sequencing, joint read/write API surface, EEF gripper-command dispatch (binary heuristic for both parallel and vacuum), SDK-error propagation.
- **EEF read/write semantics on Lite6** (since the xarm SDK doesn't expose continuous gripper state/control):
  - `read_eef_positions` / `read_eef_velocities` return `None`.
  - `write_eef_positions` thresholds at `|p| > 0.004 m` (half of the 0.008 m URDF range) → open / close.
  - `write_eef_velocities` uses sign of max element; zero → stop.
  - Module docstring documents the limitation; revisit when a continuously-actuated EEF enters the picture.
- xarm SDK calls (`set_servo_angle_j`, `vc_set_joint_velocity`, `get_joint_states`, `open/close/stop_lite6_gripper`, `set_vacuum_gripper`) all go through a `_check(ret_code, op)` helper that raises `Lite6DriverError` on non-zero status and triggers `emergency_stop()` first.

## Stubs / explicit TODOs

- **xarm SDK install** — pinned in `pyproject.toml` under `[project.optional-dependencies] hardware = ["xarm-python-sdk==1.13.0"]`. Not installed by `uv pip install --no-cache-dir -e .`; needs `uv pip install --no-cache-dir -e ".[hardware]"`. Bump the pin to `1.17.3` (current as of the time of writing) when actually moving to hardware.
- **xarm API verification against 1.17** — the driver's calls match the 1.13-era patterns from the deprecated code. The 1.17 changelog should be skimmed before driving real hardware; in particular, `set_vacuum_gripper` semantics on the Lite6 specifically.
- **Drake plant integration** — `Lite6Model` exposes the description path / frame names / DOF counts, but no helpers yet for "construct a `MultibodyPlant` from this model and weld it to the world / a mounting frame". That belongs in a follow-up plan focused on sim integration.
- **Old code** — `deprecated_lite6/` remains in place. Its tests still pass and are still in the suite; remove when all consumers (Talos backends, etc.) have migrated to the new model + driver.

  


  
