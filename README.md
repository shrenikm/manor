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

```
src/manor/           # Main package
  common/            # Shared utilities, types, exceptions, control abstractions
  lite6/             # Lite6 manipulator code (pliant control, systems, utilities)
  analysis/          # Analysis and choreography tools
  inspection/        # Visualization and debugging scripts
  algorithms/        # Manipulation algorithm implementations
models/              # URDF/SDF files for environments and objects
robot_models/        # Git submodule with Lite6 robot description files
```
