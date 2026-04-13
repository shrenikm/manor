from typing import Optional, Sequence

import numpy as np
from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, JointSliders, StartMeshcat
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import RigidBody
from pydrake.visualization import AddDefaultVisualization, AddFrameTriadIllustration

from manr.common.model_utils import ObjectModelConfig, ObjectModelType, add_object_models_to_plant
from manr.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    add_lite6_model_to_plant,
    get_default_height_for_object_model_type,
)


def visualize_manipulator(
    lite6_model_type: Lite6ModelType,
    object_model_configs: Optional[Sequence[ObjectModelConfig]] = None,
    place_on_table: bool = True,
    show_frames: bool = False,
) -> None:
    builder = DiagramBuilder()

    plant: MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0)

    lite6_model = add_lite6_model_to_plant(
        plant=plant,
        lite6_model_type=lite6_model_type,
        place_on_table=place_on_table,
    )
    object_models = add_object_models_to_plant(
        plant=plant,
        object_model_configs=object_model_configs,
    )
    plant.Finalize()
    print(plant.num_positions(lite6_model))

    meshcat.DeleteAddedControls()

    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    if show_frames:
        print("Body frames that are going to be illustrated:")
        for model_instance in [lite6_model] + object_models:
            for body_index in plant.GetBodyIndices(model_instance=model_instance):
                body: RigidBody = plant.get_body(body_index)
                print(body.name())
                # Add frame triad.
                AddFrameTriadIllustration(
                    scene_graph=scene_graph,
                    plant=plant,
                    body=body,
                    length=0.15,
                    radius=0.001,
                )

    diagram = builder.Build()
    sliders.Run(diagram, None)


if __name__ == "__main__":
    meshcat = StartMeshcat()

    lite6_model_type = Lite6ModelType.ROBOT_WITH_ARP_GRIPPER
    object_model_configs = [
        ObjectModelConfig(
            object_model_type=ObjectModelType.CUBE_1_INCH_BLUE,
            position=np.array(
                [
                    0.2,
                    0.0,
                    get_default_height_for_object_model_type(ObjectModelType.CUBE_1_INCH_BLUE),
                ]
            ),
        ),
    ]
    place_on_table = True
    show_frames = True
    visualize_manipulator(
        lite6_model_type=lite6_model_type,
        object_model_configs=object_model_configs,
        place_on_table=place_on_table,
        show_frames=show_frames,
    )
