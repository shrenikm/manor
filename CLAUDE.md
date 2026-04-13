# Project: Manor

The project is used for:
- Implementing algorithms/research ideas in manipulation (both classical and learning based)
- Experimenting with different research directions (both classical and learning based)

## Tech Stack
- Python 3.12 running through a conda environment called "manor"
- Drake robotics toolbox for control, message passing and simulation

## Hardware

- The primary hardware platform is the Ufactory Lite6 manipulator

## Project Layout

- models/ contains description files for environments/objects (URDF, SDF, etc)
- robot_models/ is a submodule that points to a project containing robot description files (URDF, SDF, etc)
- third_party/ contains third party libraries/code like robot drivers, etc
