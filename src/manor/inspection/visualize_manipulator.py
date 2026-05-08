"""
Standalone manipulator visualizer.

Loads a Lite6Model (and optional environment cubes) into a Drake plant, attaches JointSliders so
the user can drag joints around in meshcat, and renders body frames if requested. Useful for
poking at URDFs, eyeballing reachable workspace, and capturing screenshots of named poses.

Replaces the deprecated visualizer that used the old pliant stack. The new flow doesn't rely on
the aegis simulator (no Gaia / Talos / Kyber) -- it's a one-shot Drake diagram with the slider
widget and meshcat visualization.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence

import numpy as np
from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, JointSliders, StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import RigidBody
from pydrake.visualization import AddDefaultVisualization, AddFrameTriadIllustration

from manor.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
)
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant

# Demo cube z-position: half the 1-inch side length so the cube rests on the world plane (z=0).
# Used only by the __main__ demo; live code should compute heights from the actual environment.
_DEMO_CUBE_Z_M = 0.0254 / 2.0


def visualize_manipulator(
    model: Lite6Model,
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


def _main() -> None:
    parser = argparse.ArgumentParser(description="Visualize a Lite6 in meshcat with joint sliders.")
    parser.add_argument(
        "--variant",
        default=Lite6Variant.PARALLEL_GRIPPER_NORMAL.value,
        choices=[v.value for v in Lite6Variant],
        help="Lite6 variant to load.",
    )
    parser.add_argument("--show-frames", action="store_true", help="Render body-frame triads.")
    parser.add_argument("--no-cubes", action="store_true", help="Skip the demo blue cube on the table.")
    args = parser.parse_args()

    model = Lite6Model(variant=Lite6Variant(args.variant))
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
