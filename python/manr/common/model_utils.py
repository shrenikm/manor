import os
from enum import StrEnum
from typing import List, Optional, Sequence

import attr
from pydrake.multibody.parsing import PackageMap, Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.all import RigidTransform

from manr.common.custom_types import DirPath, FilePath, PositionsVector

MODELS_DIRNAME = "models"
ENVIRONMENT_MODELS_DIRNAME = "environment"
OBJECT_MODELS_DIRNAME = "objects"
ROBOT_MODELS_DIRNAME = "robot_models"
ROBOT_MODELS_DRAKE_URDF_DIRNAME = "drake_urdf"

# TODO: Tests.


class ObjectModelType(StrEnum):
    CUBE_1_INCH_RED = "cube_1_inch_red.urdf"
    CUBE_1_INCH_GREEN = "cube_1_inch_green.urdf"
    CUBE_1_INCH_BLUE = "cube_1_inch_blue.urdf"
    CUBE_1_INCH_YELLOW = "cube_1_inch_yellow.urdf"


@attr.frozen
class ObjectModelConfig:
    object_model_type: ObjectModelType
    position: Optional[PositionsVector] = None


def get_models_directory_path() -> DirPath:
    current_directory_path = os.path.dirname(os.path.expanduser(os.path.realpath(__file__)))
    models_directory_path = os.path.join(
        current_directory_path,
        "..",
        "..",
        "..",
        MODELS_DIRNAME,
    )
    return os.path.realpath(models_directory_path)


def get_environment_models_directory_path() -> DirPath:
    return os.path.join(
        get_models_directory_path(),
        ENVIRONMENT_MODELS_DIRNAME,
    )


def get_object_models_directory_path() -> DirPath:
    return os.path.join(
        get_models_directory_path(),
        OBJECT_MODELS_DIRNAME,
    )


def get_robot_models_directory_path() -> DirPath:
    current_directory_path = os.path.dirname(os.path.expanduser(os.path.realpath(__file__)))
    robot_models_directory_path = os.path.join(
        current_directory_path,
        "..",
        "..",
        "..",
        ROBOT_MODELS_DIRNAME,
    )
    return os.path.realpath(robot_models_directory_path)


def get_object_model_urdf_path(object_model_type: ObjectModelType) -> FilePath:
    return os.path.join(
        get_object_models_directory_path(),
        object_model_type.value,
    )


def add_robot_models_to_package_map(package_map: PackageMap) -> None:
    """
    Add all the robot models/descriptions inside robot_models/ into the package map.
    """
    robot_models_directory_path = get_robot_models_directory_path()

    all_robot_model_dirnames_and_paths = [
        (dirname, dirpath)
        for dirname in os.listdir(robot_models_directory_path)
        if os.path.isdir(dirpath := os.path.join(robot_models_directory_path, dirname))
    ]

    for package_name, package_path in all_robot_model_dirnames_and_paths:
        package_map.Add(
            package_name=package_name,
            package_path=package_path,
        )


def add_object_models_to_plant(
    plant: MultibodyPlant,
    object_model_configs: Optional[Sequence[ObjectModelConfig]] = None,
    parser: Optional[Parser] = None,
) -> List[ModelInstanceIndex]:
    if object_model_configs is None:
        return []

    if parser is None:
        parser = Parser(plant)

    model_instances = []

    for object_model_config in object_model_configs:
        model_instance = parser.AddModels(
            get_object_model_urdf_path(
                object_model_type=object_model_config.object_model_type,
            ),
        )[0]
        model_instances.append(model_instance)

        body = plant.get_body(body_index=plant.GetBodyIndices(model_instance=model_instance)[0])

        if object_model_config.position is not None:
            plant.SetDefaultFreeBodyPose(
                body=body,
                X_WB=RigidTransform(
                    p=object_model_config.position,
                ),
            )

    return model_instances
