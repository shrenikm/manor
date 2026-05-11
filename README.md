# Manor

Repository for manipulation research. Supports both simulation (via Drake) and real hardware.

## Setup

```bash
# Create and activate the conda environment
conda create -n manor python=3.12
conda activate manor

# Install uv if not already installed
pip install uv

# Install manor (editable)
uv pip install -e .

# For hardware support (xArm SDK)
uv pip install -e ".[hardware]"
```

## Running Tests

```bash
pytest src/ -v
```

## Project Layout

# TODO

## Additional READMEs

- [src/manor/common/aegis/README.md](src/manor/common/aegis/README.md) — aegis runtime: the multi-process observation → policy → controller → actuation loop over LCM.
- [src/manor/manipulators/README.md](src/manor/manipulators/README.md) — manipulator families, types/variants, and how they register with the rest of the codebase.
- [src/manor/manipulators/lite6/README.md](src/manor/manipulators/lite6/README.md) — empirical notes on the Ufactory Lite6 via the xArm Python SDK.

