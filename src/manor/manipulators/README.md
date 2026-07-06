# manor.manipulators

Manipulator families live here. Each family is a Python subpackage (e.g. `lite6/`) that owns its description files, variant enum, model class, driver, and any helper modules.

## Types and variants

A **type** is a manipulator family — a single piece of hardware as the manufacturer ships it. The set of supported types is the `ManipulatorType` enum in `manipulator_type.py`:

- `LITE6` — Ufactory Lite6 6-DOF arm.
- `REBOT_B601_DM` — Seeed Studio reBot Arm B601 DM, a 6-DOF Damiao-motor arm with an actuated parallel gripper.

A **variant** is a specific trim of that type. Different end-effectors, mounting orientations, or any configuration that changes the kinematic chain or the Drake URDF gets its own variant. Each type has its own variant enum, defined as a subclass of `IManipulatorVariant` in the family's `variant.py`. For Lite6 (`lite6/variant.py`) the variants are:

- `VACUUM_GRIPPER` — vacuum end-effector.
- `PARALLEL_GRIPPER_NORMAL` — actuated parallel gripper, finger heads facing inward.
- `PARALLEL_GRIPPER_REVERSE` — actuated parallel gripper, finger heads facing outward.

For the reBot B601 DM (`rebot_b601_dm/variant.py`) there is a single variant, `PARALLEL_GRIPPER`, since the arm ships in one configuration.

Cross-manipulator code (CLIs, the aegis YAML loader, the visualizer) refers to a manipulator by a `(ManipulatorType, IManipulatorVariant)` pair and never imports a concrete model class directly — that's what lets you write a single CLI flag like `--type lite6 --variant parallel_gripper_normal` and have it route to the right Drake model.

## How families register themselves

Two decorators in `manipulator_variant.py` wire a family into the registries that the rest of the codebase queries:

- `@register_manipulator_variant(ManipulatorType.X)` on the variant enum.
- `@register_manipulator_model(ManipulatorType.X)` on the concrete `IManipulatorModel` class.

Both decorators run as a side effect of importing the module they live in, so the registries are only populated for families whose modules have been imported. To make sure every family is available the moment anything touches `manor.manipulators`, the package `__init__.py` imports each family's `model` module:

```python
from manor.manipulators.lite6 import model as _lite6_model  # noqa: F401
```

The model module pulls in its variant module via its own internal imports, so a single line per family is enough to fire both decorators. Any import path that reaches into `manor.manipulators` (an `import manor.manipulators`, `from manor.manipulators.X import Y`, or even `from manor.manipulators.lite6.model import Lite6Model`) executes the package `__init__.py` first and the registry is populated before the caller's code runs.

## Adding a new manipulator type

1. Add the new member to `ManipulatorType` in `manipulator_type.py`.
2. Create a subpackage `manor/manipulators/<name>/` containing at minimum:
   - `variant.py` defining a subclass of `IManipulatorVariant` decorated with `@register_manipulator_variant(ManipulatorType.<NAME>)`, listing the trims and implementing `get_manipulator_type`.
   - `model.py` defining a concrete `IManipulatorModel` subclass decorated with `@register_manipulator_model(ManipulatorType.<NAME>)`. The class must accept a `variant=` keyword argument of the matching variant enum type. Import the variant module from inside `model.py` so a single import of `model` brings both registrations along.
   - `driver.py` if the family targets real hardware.
3. Add one line to `manor/manipulators/__init__.py`:
   ```python
   from manor.manipulators.<name> import model as _<name>_model  # noqa: F401
   ```
4. Drop URDFs / SDFs into the appropriate place under `models/` or `robot_models/` and reference them from your model class.
5. Add tests under `manor/manipulators/<name>/tests/`.

Once those are in place, every consumer (`visualize_manipulator`, the aegis YAML parser, etc.) picks up the new family automatically — no dispatch tables to update.
