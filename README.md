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
