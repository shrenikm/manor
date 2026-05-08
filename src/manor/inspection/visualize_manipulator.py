"""
Standalone manipulator visualizer.

Loads any manipulator (selected by ``ManipulatorType`` + variant) into a Drake plant, attaches
JointSliders so the user can drag joints around in meshcat, and renders body frames if
requested. Useful for poking at URDFs, eyeballing reachable workspace, and capturing screenshots
of named poses.

The manipulator family is resolved through the manipulator registry, so this script picks up new
manipulators automatically as long as they register their variant + model classes via the
``register_manipulator_variant`` / ``register_manipulator_model`` decorators in their package.
"""

from __future__ import annotations

import argparse
import importlib
import pkgutil
from collections.abc import Sequence

import numpy as np
from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, JointSliders, StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import RigidBody
from pydrake.visualization import AddDefaultVisualization, AddFrameTriadIllustration

import manor.manipulators as _manipulators_pkg
from manor.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
)
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import (
    IManipulatorVariant,
    build_manipulator_model,
    get_registered_manipulator_types,
    get_variant_class,
)

# Demo cube z-position: half a 1-inch side length so the cube rests on the world plane (z=0).
# Used only by the __main__ demo invocation; live code should compute heights from the actual
# environment.
_DEMO_CUBE_Z_M = 0.0254 / 2.0


def _import_all_manipulator_packages() -> None:
    """
    Walk the ``manor.manipulators`` namespace and import each top-level manipulator subpackage's
    ``model`` module. Each model module is where the variant + model registry decorators run, so
    importing it is what makes a manipulator family discoverable to the registry. Without this
    a fresh interpreter only sees the manipulators that some other module has already imported.
    """
    for module_info in pkgutil.iter_modules(_manipulators_pkg.__path__):
        if not module_info.ispkg:
            continue
        # Each manipulator subpackage exposes its registry-decorated classes via its `model`
        # module (and `variant` for the enum). Importing both is what registers them.
        for submodule_name in ("variant", "model"):
            try:
                importlib.import_module(f"{_manipulators_pkg.__name__}.{module_info.name}.{submodule_name}")
            except ModuleNotFoundError:
                # Some subpackages (e.g. `tests`) won't have these submodules; skip silently.
                continue


def visualize_manipulator(
    model: IManipulatorModel,
    object_model_configs: Sequence[ObjectModelConfig] | None = None,
    show_frames: bool = False,
) -> None:
    """
    Build a Drake diagram with the manipulator + optional cubes, hand control to a meshcat
    JointSliders panel. Blocks until the slider window is closed.
    """
    meshcat = StartMeshcat()

    builder = DiagramBuilder()
    plant: MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    parser = Parser(plant)
    manipulator_model_instance = parser.AddModels(model.get_description_filepath())[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(model.get_base_frame_name()))
    object_model_instances = add_object_models_to_plant(
        plant=plant,
        object_model_configs=list(object_model_configs) if object_model_configs is not None else None,
    )
    plant.Finalize()

    meshcat.DeleteAddedControls()
    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    if show_frames:
        for model_instance in [manipulator_model_instance, *object_model_instances]:
            for body_index in plant.GetBodyIndices(model_instance=model_instance):
                body: RigidBody = plant.get_body(body_index)
                AddFrameTriadIllustration(
                    scene_graph=scene_graph,
                    plant=plant,
                    body=body,
                    length=0.15,
                    radius=0.001,
                )

    diagram = builder.Build()
    sliders.Run(diagram, None)


def _resolve_manipulator(manipulator_type_value: str, variant_value: str) -> IManipulatorModel:
    try:
        manipulator_type = ManipulatorType(manipulator_type_value)
    except ValueError as e:
        valid = sorted(t.value for t in get_registered_manipulator_types())
        raise SystemExit(f"Unknown manipulator type {manipulator_type_value!r}; expected one of {valid}") from e
    variant_cls = get_variant_class(manipulator_type)
    try:
        variant: IManipulatorVariant = variant_cls(variant_value)
    except ValueError as e:
        valid = sorted(v.value for v in variant_cls)
        raise SystemExit(
            f"Unknown variant {variant_value!r} for {manipulator_type.value}; expected one of {valid}"
        ) from e
    return build_manipulator_model(manipulator_type=manipulator_type, variant=variant)


def _main() -> None:
    _import_all_manipulator_packages()

    parser = argparse.ArgumentParser(description="Visualize a manipulator in meshcat with joint sliders.")
    parser.add_argument(
        "--type",
        required=True,
        help="Manipulator type (e.g. 'lite6'). Must match a registered ManipulatorType.",
    )
    parser.add_argument(
        "--variant",
        required=True,
        help="Variant string (e.g. 'parallel_gripper_normal'). Must match a registered variant for --type.",
    )
    parser.add_argument("--show-frames", action="store_true", help="Render body-frame triads.")
    parser.add_argument("--no-cubes", action="store_true", help="Skip the demo blue cube on the table.")
    args = parser.parse_args()

    model = _resolve_manipulator(manipulator_type_value=args.type, variant_value=args.variant)
    if args.no_cubes:
        object_model_configs: list[ObjectModelConfig] = []
    else:
        object_model_configs = [
            ObjectModelConfig(
                object_model_type=ObjectModelType.CUBE_1_INCH_BLUE,
                position=np.array([0.2, 0.0, _DEMO_CUBE_Z_M]),
            ),
        ]
    visualize_manipulator(
        model=model,
        object_model_configs=object_model_configs,
        show_frames=args.show_frames,
    )


if __name__ == "__main__":
    _main()
