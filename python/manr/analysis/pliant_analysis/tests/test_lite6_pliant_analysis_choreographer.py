from manr.analysis.pliant_analysis.lite6_pliant_analysis_choreographer import (
    Lite6PliantChoreographer,
    get_choreographer_config_yaml_filepath,
)
from manr.common.testing_utils import execute_pytest_file


def test_choreographer_from_yaml() -> None:
    yaml_filepath = get_choreographer_config_yaml_filepath()
    choreographer = Lite6PliantChoreographer.from_yaml(
        yaml_filepath=yaml_filepath,
    )
    assert isinstance(choreographer, Lite6PliantChoreographer)


if __name__ == "__main__":
    execute_pytest_file()
