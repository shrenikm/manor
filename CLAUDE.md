# Project: Manor

The project is used for:
- Implementing algorithms/research ideas in manipulation (both classical and learning based)
- Experimenting with different research directions (both classical and learning based)

The project supports both: execution in simulation and real hardware

## Tech Stack
- Python 3.12 running through a conda environment called "manor"
- Drake robotics toolbox for control, message passing and simulation

## Hardware

- The primary hardware platform is the Ufactory Lite6 manipulator

## Project Layout

- src/manor/ contains the main Python package
- models/ contains description files for environments/objects (URDF, SDF, etc)
- robot_models/ is a submodule that points to a project containing robot description files (URDF, SDF, etc)

## Additional READMEs

These describe how major subsystems are set up. Read them when working in the relevant area:

- src/manor/common/aegis/README.md — aegis runtime: the multi-process observation → policy → controller → actuation loop over LCM.
- src/manor/manipulators/README.md — manipulator families, types/variants, and how they register with the rest of the codebase.
- src/manor/manipulators/lite6/README.md — empirical notes on the Ufactory Lite6 via the xArm Python SDK.
- src/manor/manipulators/rebot_b601_dm/README.md — notes on the Seeed reBot B601 DM via motorbridge, including the mandatory FORCE_POS gripper torque cap.

## Code Style

- When writing tests, please add the function to run the tests (run_manor_tests()) at the end of every file
- AVOID raw strings unless it absolutely doesn't make sense to do so (like in temporary scripts, etc)
    - Try to create global variables for strings
    - For Drake channel and port names, please create enums for the string values
