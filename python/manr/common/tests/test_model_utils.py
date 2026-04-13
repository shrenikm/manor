import os

import numpy as np
import pytest
from pydrake.multibody.parsing import PackageMap, Parser
from pydrake.multibody.plant import MultibodyPlant

from manr.common.model_utils import (
    ObjectModelConfig,
    ObjectModelType,
    add_object_models_to_plant,
    add_robot_models_to_package_map,
    get_environment_models_directory_path,
    get_models_directory_path,
    get_object_model_urdf_path,
    get_object_models_directory_path,
    get_robot_models_directory_path,
)
from manr.common.path_utils import list_directories_in_path
from manr.common.testing_utils import execute_pytest_file


def test_get_models_directory_path() -> None:
    path = get_models_directory_path()
    assert os.path.exists(path)
    assert os.path.basename(path) == "models"


def test_get_environment_models_directory_path() -> None:
    path = get_environment_models_directory_path()
    assert os.path.exists(path)
    assert os.path.basename(path) == "environment"


def test_get_object_models_directory_path() -> None:
    path = get_object_models_directory_path()
    assert os.path.exists(path)
    assert os.path.basename(path) == "objects"


def test_get_robot_models_directory_path() -> None:
    path = get_robot_models_directory_path()
    assert os.path.exists(path)
    assert os.path.basename(path) == "robot_models"


def test_get_object_model_urdf_path() -> None:
    for object_model_type in ObjectModelType:
        filepath = get_object_model_urdf_path(
            object_model_type=object_model_type,
        )
        assert os.path.isfile(filepath)
        assert os.path.basename(filepath) == object_model_type.value


def test_add_robot_models_to_package_map() -> None:
    package_map = PackageMap()
    add_robot_models_to_package_map(package_map=package_map)

    # More packages should be added now.

    for directory in list_directories_in_path(get_robot_models_directory_path()):
        assert package_map.Contains(directory)
        assert package_map.GetPath(directory) == os.path.join(
            get_robot_models_directory_path(),
            directory,
        )


@pytest.mark.parametrize("with_parser", [True, False])
@pytest.mark.parametrize("with_position", [True, False])
def test_add_object_models_to_plant(
    with_parser: bool,
    with_position: bool,
) -> None:
    rng = np.random.RandomState(7)
    configs = []
    for object_model_type in ObjectModelType:
        position = None
        if with_position:
            position = rng.randn(3)

        config = ObjectModelConfig(
            object_model_type=object_model_type,
            position=position,
        )
        configs.append(config)

    plant = MultibodyPlant(time_step=0.0)
    parser = None
    if with_parser:
        parser = Parser(plant)

    instances = add_object_models_to_plant(
        plant=plant,
        object_model_configs=configs,
        parser=parser,
    )

    # Test that each config added returns a model instance.
    assert len(instances) == len(configs)

    # Test that each model has been added to the plant.
    for object_model_type in ObjectModelType:
        name = object_model_type.value.split(".")[0]
        assert plant.HasModelInstanceNamed(name=name)


if __name__ == "__main__":
    execute_pytest_file()
