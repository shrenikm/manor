"""
Standalone manipulator visualizer.

Loads any manipulator (selected by ManipulatorType + variant) into a Drake plant, attaches JointSliders so the user can
drag joints around in meshcat, and renders body frames if requested. Useful for poking at URDFs, eyeballing reachable
workspace, and capturing screenshots of named poses.

The manipulator family is resolved through the manipulator registry. Importing manor.manipulators is what populates
that registry; see manor/manipulators/README.md for how new families plug themselves in.
"""

from __future__ import annotations

import argparse

from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, JointSliders
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import RigidBody
from pydrake.visualization import AddDefaultVisualization, AddFrameTriadIllustration

import manor.manipulators  # noqa: F401  -- side-effect import populates the manipulator registry.
from manor.common.meshcat_utils import start_meshcat
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import (
    IManipulatorVariant,
    build_manipulator_model,
    get_registered_manipulator_types,
    get_variant_class,
)


def visualize_manipulator(model: IManipulatorModel, show_frames: bool = False) -> None:
    """
    Build a Drake diagram with the manipulator and hand control to a meshcat JointSliders panel. Blocks until
    the slider window is closed.
    """
    meshcat = start_meshcat()

    builder = DiagramBuilder()
    plant: MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    parser = Parser(plant)
    add_robot_models_to_package_map(parser.package_map())
    manipulator_model_instance = parser.AddModels(model.get_description_filepath())[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(model.get_base_frame_name()))
    plant.Finalize()

    meshcat.DeleteAddedControls()
    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    if show_frames:
        for body_index in plant.GetBodyIndices(model_instance=manipulator_model_instance):
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
    args = parser.parse_args()

    model = _resolve_manipulator(manipulator_type_value=args.type, variant_value=args.variant)
    visualize_manipulator(model=model, show_frames=args.show_frames)


if __name__ == "__main__":
    _main()
