"""
Cap'n Proto loading and numpy <-> Float64Array conversion helpers.

Each definition has a single schema file at definitions/schemas/capnp/<name>.capnp
containing the per-version structs AND the versioned-union wrapper.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

import capnp
import numpy as np

_SCHEMA_ROOT = Path(__file__).resolve().parents[1] / "schemas" / "capnp"


@lru_cache(maxsize=None)
def load_versioned_schema(definition_name: str) -> Any:
    """
    Load the schema file for a definition.

    E.g. load_versioned_schema("timestamp_header") loads
        schemas/capnp/timestamp_header.capnp
    and returns the pycapnp module exposing TimestampHeaderV1 and
    VersionedTimestampHeader.

    A fresh SchemaParser is used per definition to avoid duplicate-ID errors
    when the shared default parser sees the same transitively-imported schema
    file through multiple definitions.
    """
    parser = capnp.SchemaParser()
    schema_path = _SCHEMA_ROOT / f"{definition_name}.capnp"
    return parser.load(
        str(schema_path),
        imports=[str(_SCHEMA_ROOT)],
    )


def ndarray_to_float64_array(arr: np.ndarray, builder: Any) -> None:
    """
    Populate a Cap'n Proto Float64Array builder from a numpy ndarray.
    """
    contiguous = np.ascontiguousarray(arr, dtype=np.float64)
    builder.shape = [int(s) for s in contiguous.shape]
    builder.data = contiguous.reshape(-1).tolist()


def float64_array_to_ndarray(reader: Any) -> np.ndarray:
    """
    Reconstruct a numpy ndarray from a Cap'n Proto Float64Array reader.
    """
    shape = tuple(int(s) for s in reader.shape)
    flat = np.array(list(reader.data), dtype=np.float64)
    if not shape:
        return flat
    return flat.reshape(shape)
