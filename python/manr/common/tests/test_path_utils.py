import os

from manr.common.path_utils import (
    create_directory_if_not_exists,
    create_temporary_directory,
    create_temporary_file,
    list_directories_in_path,
    list_files_in_path,
)
from manr.common.testing_utils import execute_pytest_file


def test_create_temporary_directory() -> None:
    with create_temporary_directory() as temp_dirpath:
        assert os.path.isdir(temp_dirpath)

    # Directory should be deleted after the context manager exits.
    assert not os.path.isdir(temp_dirpath)

    # Check arguments.
    prefix = "a"
    suffix = "b"

    with create_temporary_directory() as dirpath:
        with create_temporary_directory(
            prefix=prefix,
            suffix=suffix,
            dirpath=dirpath,
        ) as temp_dirpath:
            assert os.path.isdir(temp_dirpath)
            assert temp_dirpath.startswith(dirpath)
            assert os.path.basename(temp_dirpath).startswith(prefix)
            assert os.path.basename(temp_dirpath).endswith(suffix)

        assert not os.path.isdir(temp_dirpath)
    assert not os.path.isdir(dirpath)


def test_create_temporary_file():
    with create_temporary_file() as temp_filepath:
        assert os.path.exists(temp_filepath)

    # File should be deleted after the context manager exits.
    assert not os.path.exists(temp_filepath)

    prefix = "a"
    suffix = ".b"

    with create_temporary_directory() as dirpath:
        with create_temporary_file(
            prefix=prefix,
            suffix=suffix,
            dirpath=dirpath,
        ) as temp_filepath:
            assert os.path.isfile(temp_filepath)
            assert temp_filepath.startswith(dirpath)
            assert os.path.basename(temp_filepath).startswith(prefix)
            assert os.path.basename(temp_filepath).endswith(suffix)


def test_list_directories_in_path():
    with create_temporary_directory() as temp_dirpath:
        assert len(list_directories_in_path(temp_dirpath)) == 0

        # Create a few directories and files within the temporary directory.
        os.makedirs(os.path.join(temp_dirpath, "a"))
        os.makedirs(os.path.join(temp_dirpath, "b"))
        os.makedirs(os.path.join(temp_dirpath, "c"))
        os.open(os.path.join(temp_dirpath, "d"), os.O_CREAT)
        os.open(os.path.join(temp_dirpath, "e"), os.O_CREAT)

        assert set(list_directories_in_path(temp_dirpath)) == {"a", "b", "c"}


def test_list_files_in_path():
    with create_temporary_directory() as temp_dirpath:
        assert len(list_files_in_path(temp_dirpath)) == 0

        # Create a few directories and files within the temporary directory.
        os.makedirs(os.path.join(temp_dirpath, "a"))
        os.makedirs(os.path.join(temp_dirpath, "b"))
        os.open(os.path.join(temp_dirpath, "d"), os.O_CREAT)
        os.open(os.path.join(temp_dirpath, "e"), os.O_CREAT)
        os.open(os.path.join(temp_dirpath, "f"), os.O_CREAT)

        assert set(list_files_in_path(temp_dirpath)) == {"d", "e", "f"}


def test_create_directory_if_not_exists() -> None:
    with create_temporary_directory() as temp_dirpath:
        dirpath = os.path.join(temp_dirpath, "a")
        create_directory_if_not_exists(dirpath=dirpath)

        assert os.path.isdir(dirpath)

        # Should be able to call it again, even if the directory already exists.
        create_directory_if_not_exists(dirpath=dirpath)


if __name__ == "__main__":
    execute_pytest_file()
