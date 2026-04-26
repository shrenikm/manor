# Plan 2 for Manor

Continuing the full redesign of the Manor project. Now that we have all of our definitions for the messages, we can start using them to build out our system.

The goal is to build individual sub-systems (using Drake) that will publish and subscribe to messages. Each system will have a well-defined objective and together, the entire system will be able
to run algorithms/models for robotic manipulation both in simulation and on hardware.

The full system is going to be called "Aegis". Here are the individual sub-systems:

## Helios

This is the sub-system that observes the environment and publishes sensor messages.

- Inputs: The does not subscribe to any messages
- Outputs: The system publishes messages (one per sensor). So RGB images, RGBD images, etc.
- Sim: For simulation, the system needs to publish simulated sensor data, observing a simulated environment.
- Hardware: For hardware, the system needs to use the sensor API to publish messages

### Design

- The system is a Drake System that has no input ports and as many output ports as the number of sensor messages that need to be published
- The system takes in a publish_frequency param that is used to publish messages at that frequency

## Talos

This is the sub-system that executes the actual control commands on the robot hardware (or in simulation). It also publishes the internal joint and gripper state of the robot.

- Inputs: Subscribes to the channel that publishes command messages
- Outputs: Publishes joint and gripper state messages
- Sim: For simulation, the system sends commands and moves the robot in simulation
- Hardware: For hardware, the system uses the robot APIs to send commands to move the robot

### Design

- The system is a Drake System that has one input port for the command messages and output port/ports for the joint/gripper states (maybe joint and gripper states need to be refactored into one)
- The system takes in a publish_frequency param that is used to publish messages at that frequency
- The system may also have some start and end hooks to prep the manipulator - Start at a specific configuration before executing policies and end at a specific configuration, etc.

## Soma

This is the sub-system that outputs the full proprioception state of the robot.

- Inputs: Subscribes to the joint and gripper state channel (published by Talos)
- Outputs: Performs forward kinematics to compute the EEF pose/velocities and then combines it with the input and publishes the full proprioception state message
- Sim: For simulation, the system uses the internal model (URDF, SDF, etc) to perform forward kinematics and output the full proprioception state
- Hardware: For hardware, it does the same thing -- uses the internal model (URDF, SDF, etc) to perform forward kinematics and output the full proprioception state

So for this system, there isn't much of a difference between sim and hardware

### Design

- The system is a Drake system that has input ports for the joint and gripper states and has one output port to publish the full proprioception state messages
- The system takes in a publish_frequency param that is used to publish messages at that frequency. Note that this is independent of the frequency of the messages in the inputs

## Metis

This is the sub-system that takes in the sensor messages and proprioception and outputs actions. This can be a model (learned policy) or a classical planning algorithm.

- Inputs: Subscribes to sensor messages and proprioception messages
- Outputs: Publishes actions
- No difference between sim and hardware here as the system just needs the observations to output the action

### Design

- The system is a Drake system that has input ports for the sensor messages and proprioception messages and has one output port to publish action messages
- The system takes in a publish_frequency param that is used to publish messages at that frequency. Note that this is independent of the frequency of the messages in the inputs.
- The system will hold some sort of policy protocol that will define the type of algorithm that produces the actions. Some examples are:
    - Classical motion planning
    - Trajectory optimization
    - Diffusion policies
    - VLAs

## Kyber

This is the sub-system that performs the role of a low level controller. It takes in actions and publishes commands.

- Inputs: Subscribes to a channel for actions (coming from Metis) and also the channel for full proprioception state (published by Soma)
- Outputs: Feeds the actions into a controller and publishes lower level commands
- No difference between sim and hardware here either as all the information we require for the controller are in the input actions and the current state

### Design

- The system is a Drake system that has input ports for proprioception state and the actions and has an output port to publish command messages
- The system takes in a publish_frequency param that is used to publish messages at that frequency. Note that this is independent of the frequency of the messages in the inputs
- The system will hold some sort of controller protocol object which defines what kind of control we want. Some examples are:
    - Trajetory tracking of trajectories in the action
    - Controller for differential IK
    - PID
    - LQR
    - etc

So for the next stage of the project, we will implement this design for aegis.

We want an "aegis" directory inside common/ under which we will have sub-directories for each of the sub-systems. The idea is that this scales for any robot and we can run our policies
on hardware or in sim with the switch of a flag.

# Implemented

## Final state

The Aegis stack lives at `src/manor/common/aegis/` and contains **four sub-systems** plus a top-level builder. Each sub-system is a Drake `LeafSystem`, and they all follow the same template:

- **Inputs** are abstract-valued ports carrying typed Plan 1 messages.
- **State** is one or more `DeclareAbstractState` slots holding the latest produced message.
- **Outputs** are abstract-valued ports that are pure reads of state, with `prerequisites_of_calc={self.abstract_state_ticket(...)}` so outputs are **not** direct-feedthrough on inputs — this is what lets the cyclic graph build without algebraic-loop errors.
- **Periodic ticks** via `DeclarePeriodicUnrestrictedUpdateEvent(period=1/f, ...)` recompute state from the latest input port values; outputs are zero-order holds between ticks. Each system's publish rate is decoupled from its inputs' rates.
- **Port names** are `StrEnum`s per system (`KyberPorts.INPUT_ACTION`, etc.). The only raw string port-name literals anywhere in the tree are the values inside those enum definitions.
- Every test file ends with `if __name__ == "__main__": run_manor_tests()` so individual files run as `python test_X.py`.

### Sub-systems

- **Helios** (`helios/`) — sensor publisher. No inputs; outputs `rgb_image` + `depth_image`. Takes a `SensorBackend` protocol object.
- **Talos** (`talos/`) — manipulator interface. Input `command`; output `proprioception`. Takes a `ManipulatorBackend` protocol object plus an optional `robot_model_path` (intended for FK). Forward kinematics lives here (it was Soma's job originally; see "Deviations").
- **Metis** (`metis/`) — policy runner. Inputs `proprioception`, `rgb_image`, `depth_image`; output `action`. Takes a `Policy` protocol object. One concrete `IdentityPolicy` (mirrors current joint positions back) ships in `metis/policies.py`.
- **Kyber** (`kyber/`) — low-level controller. Inputs `action`, `proprioception`; output `command`. Currently a passthrough (joint positions in → joint positions out); the `Controller` protocol is deferred.

### Sim/hardware split — backend injection

The plan called for "switch of a flag" between sim and hardware. Implemented as **backend injection**: each system that touches the world (Helios, Talos) takes a backend protocol object in its constructor. The `LeafSystem` itself is shape-identical in both modes; only the backend class changes.

- `SensorBackend` protocol → `SimSensorBackend`, `HardwareSensorBackend` (in `helios/sim_backend.py`, `helios/hardware_backend.py`).
- `ManipulatorBackend` protocol → `SimManipulatorBackend`, `HardwareManipulatorBackend` (in `talos/sim_backend.py`, `talos/hardware_backend.py`).
- All four backends are currently stubs (clean APIs, no real SDK or Drake plant calls yet).

This keeps the Drake graph topology identical across modes — important for porting policies between sim and hardware later.

### Top-level builder

- `aegis.py` exposes `build_aegis(mode, policy, robot_model_path, frequencies)` returning `(Diagram, AegisSystems)`.
- `AegisMode` is a `StrEnum` of `SIM` / `HARDWARE`; `AegisFrequencies` carries per-system Hz defaults (helios 30, talos 200, metis 10, kyber 500); `AegisSystems` is an `attr.frozen` container of the four leaf-system handles.
- Wiring after the Soma removal:
  - `Helios.{rgb,depth}_image → Metis.{rgb,depth}_image`
  - `Talos.proprioception → {Metis, Kyber}.proprioception`
  - `Metis.action → Kyber.action`
  - `Kyber.command → Talos.command`

## Deviations from the plan

- **Soma was removed.** The user folded its FK responsibility into Talos. The Soma sub-package is gone; Talos owns `_compute_eef_pose` / `_compute_eef_twist` (still stubs) and publishes `Proprioception` directly instead of separate joint + EEF state ports. Net effect: 4 sub-systems instead of 5, simpler wiring, no semantic loss.
- The plan listed Helios as publishing "RGB images, RGBD images, etc.". The implementation publishes RGB + Depth separately. RGBD is a derived combined message that can be added later if needed.

## Beyond the plan

### Shared message constructors

`src/manor/common/definitions/utils/defaults.py` — `construct_default_*()` factories for every Plan 1 message, plus `construct_zero_header()` and `construct_system_time_header()` (live wall-clock via `time.monotonic_ns()` + `time.time_ns()`). These are used as Drake `AbstractValue` model values throughout the Aegis tree, and to stamp freshly-produced outputs.

### LCM connectivity prototype

The plan notes that some sub-systems (Metis especially) need to run on a separate machine. A working LCM-bridge prototype lives in `kyber/`:

- `kyber/lcm_source.py` — standalone Drake diagram that publishes fresh `Proprioception` + `Action` LCM messages at a configurable rate (default 10 Hz). Runs as `python -m manor.common.aegis.kyber.lcm_source`.
- `kyber/kyber_lcm.py` — standalone Drake diagram that subscribes to those channels, runs a real `Kyber`, and publishes `Command` LCM messages at its publish frequency (default 50 Hz, deliberately distinct from the source). Runs as `python -m manor.common.aegis.kyber.kyber_lcm`.
- Channels: `AEGIS_PROPRIOCEPTION`, `AEGIS_ACTION`, `AEGIS_COMMAND`.
- Internal helpers `_AegisToLcmMessageSystem` / `_LcmToAegisMessageSystem` bridge between `attrs` messages and generated LCM types via the existing `to_lcm_message()` / `from_lcm_message()` methods. These helpers are duplicated across the two files for now; when the other systems get LCM-ified they should be hoisted to a shared module under `aegis/`.
- `tests/test_kyber_lcm.py` — in-process LCM round-trip tests using `DrakeLcm("memq://")`. Three tests: schema round-trip, end-to-end through Kyber, and a frequency sanity check.

### lcm-spy with decoded Aegis messages

- `scripts/compile_messages.py` was extended with `_compile_lcm_java_bindings()`. When `javac` + `jar` + `lcm.jar` are all available, it runs `lcm-gen --java`, compiles the Java sources, and bundles them into `build/java/manor_lcmtypes.jar`. Soft-skips with a warning when any tool is missing, so installs on headless / Java-less environments still succeed.
- `manor_lcm_spy` console script (`src/manor/common/manor_lcm_spy.py`, exposed via `[project.scripts]`) — exports `CLASSPATH=build/java/manor_lcmtypes.jar` and execs `lcm-spy`. The conda-shipped `lcm-spy` script appends `$CLASSPATH` to its internal classpath, so this is enough for decoded message contents in the GUI.

### Tests

- 32 aegis-tree pytest tests covering construction, periodic publish, backend protocol compliance, FK assembly, and a parametrized SIM + HARDWARE smoke run of the full diagram (`test_aegis.py::TestBuildAegis::test_diagram_advances_without_error[sim|hardware]`).

## Stubs / explicit TODOs

Each is intentionally scoped so the real implementation is a single-location change:

- **Backends** — all four (`Sim/Hardware × Sensor/Manipulator`) return zero-valued defaults. Real RealSense / Orbbec / Lite6 / Drake `MultibodyPlant` integration is a separate effort.
- **Talos FK** — `_compute_eef_pose`, `_compute_eef_twist` return identity pose / zero twist. Loading the kinematic model from `robot_model_path` and running FK lives entirely in those two methods.
- **Kyber controller protocol** — currently joint-positions passthrough; a `Controller` protocol replacing the inline logic is deferred.
- **Metis policies** — only `IdentityPolicy` exists; classical motion planners, trajectory optimization, diffusion policies, and VLAs are each their own follow-up plan.
- **Talos start/end safe-config hooks** — declared on `ManipulatorBackend` (`start()` / `stop()`) but not yet invoked from Drake events; will be triggered by higher-level orchestration.

