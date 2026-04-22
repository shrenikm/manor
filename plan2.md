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
