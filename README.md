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

- `src/manor/` — main Python package.
  - `common/aegis/` — multi-process manipulation runtime (observation → policy → controller → actuation, over LCM). Runs unchanged in sim and on hardware. See [its README](src/manor/common/aegis/README.md).
  - `common/control/`, `common/definitions/` — shared control signals and message/definition types used across the stack.
  - `manipulators/` — manipulator families (types and variants) and the registry the rest of the codebase queries by `(type, variant)`. See [its README](src/manor/manipulators/README.md).
  - `manipulators/lite6/` — Ufactory Lite6 model, driver, and CLI. Hardware-side quirks documented in [its README](src/manor/manipulators/lite6/README.md).
  - `inspection/` — visualization and inspection utilities.
- `configs/` — YAML configs consumed at runtime (`aegis/` for the aegis stack, `choreographer/` for joint choreographer sequences).
- `models/` — URDF/SDF assets for environments and objects (not robot bodies).
- `robot_models/` — git submodule of robot description files.
- `scripts/` — packaging and build-time helpers (e.g. LCM message compilation).
- `results/` — run output (plots, recordings); gitignored.

