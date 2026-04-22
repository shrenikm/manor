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

## Code Style

- When writing tests, please add the function to run the tests (run_manor_tests()) at the end of every file
