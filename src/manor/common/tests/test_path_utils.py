import os

from manor.common.path_utils import (
    create_directory_if_not_exists,
    create_temporary_directory,
    create_temporary_file,
    get_project_root,
    list_directories_in_path,
    list_files_in_path,
    resolve_under_project_root,
)
from manor.common.testing_utils import run_manor_tests


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


def test_get_project_root_returns_directory_with_pyproject() -> None:
    root = get_project_root()
    assert os.path.isdir(root)
    assert os.path.isfile(os.path.join(root, "pyproject.toml"))


def test_get_project_root_is_stable_across_calls() -> None:
    # Cached after first computation; same string both times.
    assert get_project_root() == get_project_root()


def test_resolve_under_project_root_passes_absolute_paths() -> None:
    abs_path = "/tmp/somewhere/file.yaml"
    assert resolve_under_project_root(abs_path) == abs_path


def test_resolve_under_project_root_joins_relative_paths() -> None:
    relative = "configs/something/file.yaml"
    resolved = resolve_under_project_root(relative)
    assert resolved == os.path.normpath(os.path.join(get_project_root(), relative))


if __name__ == "__main__":
    run_manor_tests()
