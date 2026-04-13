"""
Utilities for working with paths.
"""

import os
import shutil
import tempfile
from contextlib import contextmanager
from typing import Generator, List, Optional

from manr.common.custom_types import DirPath, FilePath


@contextmanager
def create_temporary_directory(
    prefix: Optional[str] = None,
    suffix: Optional[str] = None,
    dirpath: Optional[DirPath] = None,
) -> Generator[DirPath, None, None]:
    """
    Creates a temporary directory, yields the path to the directory, and then deletes the directory.
    """

    temp_dirpath = tempfile.mkdtemp(
        prefix=prefix,
        suffix=suffix,
        dir=dirpath,
    )
    try:
        yield temp_dirpath
    finally:
        shutil.rmtree(temp_dirpath)


@contextmanager
def create_temporary_file(
    prefix: Optional[str] = None,
    suffix: Optional[str] = None,
    dirpath: Optional[DirPath] = None,
) -> Generator[FilePath, None, None]:
    """
    Creates a temporary file, yields the path to the file, and then deletes the file.
    """

    fd, temp_filepath = tempfile.mkstemp(
        prefix=prefix,
        suffix=suffix,
        dir=dirpath,
    )
    os.close(fd)
    try:
        yield temp_filepath
    finally:
        os.remove(temp_filepath)


def list_directories_in_path(
    dirpath: DirPath,
) -> List[DirPath]:
    """
    Returns a list of all directories in the given path.
    """
    return [
        file_or_directory
        for file_or_directory in os.listdir(dirpath)
        if os.path.isdir(os.path.join(dirpath, file_or_directory))
    ]


def list_files_in_path(
    dirpath: DirPath,
) -> List[FilePath]:
    """
    Returns a list of all files in the given path.
    """
    return [
        file_or_directory
        for file_or_directory in os.listdir(dirpath)
        if os.path.isfile(os.path.join(dirpath, file_or_directory))
    ]


def create_directory_if_not_exists(
    dirpath: DirPath,
) -> None:
    """
    Creates a directory at the given path if it does not already exist.
    """
    os.makedirs(dirpath, exist_ok=True)
