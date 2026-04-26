# Plan 4 for Manor

Now it's time to piece the individual systems together!

Here's my vision for this:

1. We avoid unnecessary lcm publishing/subscribing as it adds latency for no reason
2. Systems that can be directly connected should be directly connected
3. With this in mind, we also have certain constraints. I'm primarily going to be testing learning based models/policies through this project (run in Metis). To run this I need to run it through a GPU for which I need to use another machine on the same network. This means that Metis needs to run as a separate block and must subscribe/publish messages without direct connections
4. We could have an optional thing where Metis can be directly connected if we're running classical planning algorithms, but I don't want to have diverging logic here and complicate things. It isn't like LCM is that slow either
5. So we go off the assumption that the observation contents (images, proprioception, etc) must be published onto channels that Metis subscribes to. Metis will also publish actions through a channel
6. Logically, it follows that Helios should publish the images and is a separate block itself
7. Applyig more logic, Metis publishes actions, so we need Kyber to subscribe to actions and this Metis-Kyber connection is not a direct connection but through LCM
8. Kyber outputs commands but this doesn't need to be published as we can directly connect this to Talos as the robot hardware control and simulation runs on my local machine as it isn't too heavy. So the command output of Kyber is wired directly to the command input of Talos.
9. Talos needs to take in the commands and output proprioception. Proprioceptoin needs to be published for Metis, so it follows that Talos publishes this through LCM
10. For hardware, the outcome is that I should be able to run different policies in Metis and see the robot move in real life
11. For simulation, I want to be able to see the simulation through Meshcat while it's running.
12. Maybe it'll be cool to have a "both" option where I run on real robot while also being able to see what's happening in simulation. But this is less of simulation, and more of take the joint state from the hardware and display it on Meshcat.

We now go through each isolated block for both real robot and simulation. I'm less sure of the simulation one so let's start with the hardware design now:

## Block 1:

This is just Helios. It has no inputs (maybe future user input to publish language instructions for VLAs but none for now). It just publishes images (RGB, RGBD) as lcm messages

## Block 2:

This is just Metis. It subscribes to the image lcm messages and the proprioception lcm messages and publishes action lcm messages

## Block 3:

This is Kyber + Talos. Kyber subscribes to the action lcm messages and outputs commands which are directly wired to the command input of Talos. Talos outputs proprioception which is published as lcm messages for Metis to subscribe to.
Notes:
1. Kyber will likely need to do inverse kinematics (for going from EEF actions to joint commands, etc). So we'd need to do something like differential IK (or even trajectory tracking) which means that we would need to hold an IManipulatorModel instance to do this. We would need to create a Drake plant inside (I think?) to do all of this.
2. Talos will need to do forward kinematics to get the eef pose, twist (as we only get the joint states from the ManipulatorDriver and we need eef information for the full proprioception). Obviously this also means that we need the drake model loaded here as well to do FK


Now for simulation. I need to make some design decision here.

## Block 2:

This is the Metis block and is unchanged.

## Block 1/3 (Some questions):

The kyber + Talos wiring is unchanged for simulation. The key thing here is:
1. Kyber will be similar - same model requirement to do diffIk, trajectory tracking if required etc.
2. Talos is different. Instead of the ManipulatorDriver writing commands to hardware, we need to send commands to a simulation. So the same interfaces for sending joint positions, velocities, etc must be rerouted to simulation. We then also need to get the proprioception state from the simulation and publish this. This differs from the real robot

So here's my open ended question: Where do we want this simulation thing to lie and what to do with Helios?

### Option 1

Gaiaulation happens in Talos. 

- 1a. We could have Talos publish images and not have a separate helios for simulation. To publish images, we need a camera setup in the simulation. This is fine, but I really don't want Talos publishing images as the design gets weird even if it's just for sim.
- 1b. Not sure if possible, but we could have helios as another unit that purely subscribes to the simulation happening and just does the rendering. So the actual physics simulation happens in Talos, and we have a direct connection to Helios which can then render and pulish image messages

### Option 2 

Gaiaulation happens in a separate block.

- 2a. Talos sends commands to the sim block and the sim block does the simulation and rendering. It then connects directly into helios and helios simply forwards and publishes the images it gets from the sim renderings
- 2b. The sim block is just Helios and it directly takes in commands from Talos to run the sim and publishes the rendered images.

Note, I'm not really that concerned about rendered images:
1. Drake isn't particularly good at realistic renderings
2. I'm not likely to run policies in sim anytime soon. I will probably only use sim to test the control pipeline

But I still want to set this up now for completeness and making sure that future design decision are dealing with this hole.

* Also note that for sim, I want to use an environment similar to my actual set up which is the robot mounted on a table. If you notice, I've been using this table urdf in the old code, and I construct the drake multibodyplant with manipulator + table. I'd like to do it similar, but it needs to be more flexible/generic. I don't want to always do this table thing as it needs to be extendible to other environments and manipulators. So ideally we need to design some kind of configuration file (yaml or something?) which can define the environment the robot will be active in and we can use this in Kyber, Talos, etc.
* This isn't super important for now, but it would be nice if our design choices now nicely allowed for the "both" option where we can run on real robot while also displaying the robot states in sim. 
* Let me know your thoughts here.

# Implementation

## Design (decided)

- **Gaia-as-Python-object** (not a Drake LeafSystem). A `Gaia` class lives at
  `src/manor/common/aegis/sim/sim.py`. It owns its own internal `MultibodyPlant`,
  `SceneGraph`, `RgbdSensor`s, `MeshcatVisualizer`, and an internal
  `Gaiaulator`. Its public API is plain Python:
    - `apply_joint_position_command(positions)`
    - `apply_joint_velocity_command(velocities)`
    - `read_joint_state() -> JointState`
    - `render_rgb(camera_id) -> RGBImageData`
    - `render_depth(camera_id) -> DepthImageData`
    - `advance_to(t)`
    - `set_joint_positions(positions)`  (used by the future "both" mirror mode)
- **Gaia/hardware graph-shape parity.** The outer Aegis `Diagram` has the
  same systems and same wiring in both modes. Only the backends (sim vs
  hardware) and a sim-only `GaiaAdvancer` differ.
- **`GaiaAdvancer` LeafSystem** (sim-mode only): one extra LeafSystem with
  a fast (~500 Hz) periodic event that calls `sim.advance_to(context.get_time())`.
  Gaia is *driven* by the diagram's clock; backends just *read* it.
- **`EnvironmentConfig`**: attrs class loaded from YAML. Describes the
  manipulator's mounting frame and any extra static models welded into
  the world. Default (no env): manipulator welded to world origin.
  Lives at `src/manor/common/aegis/sim/env_config.py`.
- **Each system owns its own plant.** Kyber holds an `IManipulatorModel`
  to build its own MultibodyPlant for diff-IK / trajectory tracking.
  Talos holds its own MultibodyPlant for FK (eef pose + twist). Gaia has
  its own physics plant. None are shared. Same URDFs, three independent
  plant instances.
- **Block 1/2/3 nomenclature is just for explanation** in the plan; the
  code does not reflect it.

## Stubs adopted in this phase

To get the high-level wiring + architecture in place without front-loading
the algorithmic work:

- **Metis policy stub**: emits a zero-velocity `JointVelocities` action.
- **Kyber controller stub**: emits a zero-velocity `JointVelocities`
  command (passes the action through, conceptually). No diff-IK yet.
- **Talos FK stub**: still returns identity pose + zero twist; the
  per-Talos `MultibodyPlant` plumbing is added but FK math is deferred.
- **Gaia backend stubs**: render-paths return empty image buffers; physics
  is zero-actuation (the position command is held but not yet applied to
  the plant's actuation input). The Drake plant + `Gaiaulator` *are*
  constructed and `advance_to` *is* called every tick, so the structural
  seam exists.

## Wiring (final, both modes)

Same in sim and hardware -- only the backends differ.

```
Helios.OUTPUT_RGB_IMAGE        --> AegisLCMPublisherAdapter[RGB_IMAGE]
Helios.OUTPUT_DEPTH_IMAGE      --> AegisLCMPublisherAdapter[DEPTH_IMAGE]
Talos.OUTPUT_PROPRIOCEPTION    --> AegisLCMPublisherAdapter[PROPRIOCEPTION]

AegisLCMSubscriberAdapter[RGB_IMAGE]      --> Metis.INPUT_RGB_IMAGE
AegisLCMSubscriberAdapter[DEPTH_IMAGE]    --> Metis.INPUT_DEPTH_IMAGE
AegisLCMSubscriberAdapter[PROPRIOCEPTION] --> Metis.INPUT_PROPRIOCEPTION

Metis.OUTPUT_ACTION                       --> AegisLCMPublisherAdapter[ACTION]
AegisLCMSubscriberAdapter[ACTION]         --> Kyber.INPUT_ACTION

Kyber.OUTPUT_COMMAND          --> Talos.INPUT_COMMAND   (direct, not LCM)
```

Gaia-only: `GaiaAdvancer` LeafSystem in the same diagram, no input/output
ports, periodic event drives `sim.advance_to`.

## Layout

```
src/manor/common/aegis/
  ... (existing helios, talos, metis, kyber, adapters, builder)
  sim/
    __init__.py
    sim.py            -- Gaia class
    env_config.py     -- EnvironmentConfig + YAML loader
```

The existing `src/manor/common/aegis/` (helios, talos, metis, kyber,
adapters, builder) stays where it is; only the *backends* and the
*builder* gain knowledge of Gaia.
