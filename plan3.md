# Plan 3 for Manor

Continuing with the full redesign of the project, the next step is to write the new interfaces for the manipulator itself.
Note that I have renamed manor/manipulators/lite6 to manor/manipulators/deprecated_lite6 to make room for the upgrade. I dislike the old way of doing things and want a fresh refactor/redesign.

The object is to be flexible enough to support different manipulators and different model types of the same manipulator.
Currenlty there is only one manipulator --  the Ufactory Lite6 robot 
This has a bunch of different models/trims (with vaccum gripper, without any grippers, with parallel grippers in different orientations, etc)

Currently in Lite6ModelType there are 6 different models, but I want to reduce them and only use the necessary ones.
I wnat to get rid of the vacuum only models and the unactuated models. So we're left with 3: vacuum gripper, parallel gripper in the two configurations (normal and reverse)
Let's keep all of the existing urdfs from the other robot_models project and just map them differently to the model types (only the actuated ones will be used in Manor)

First, we need two enums:

class ManipulatorType(StrEnum):
  LITE6 = "lite6"

This is the base/top level enum for manipulator type. We also have a manipulator model type for the individual models/trims of the manpulator:

class ManipulatorModelType(StrEnum):
  
  @abstractmethod
  def get_manipulator_type(self) -> ManipulatorType:
    ...

class Lite6ModelType(ManipulatorModelType):
  ...

This will be the same enum as the existing deprecated Lite6ModelType but without the vacuum only models (so 6 types in total)

Next we need two interfaces, one of the model to get the characteristics of the manipulator (for sim, etc) and one for the hardware interface to actually move a physical robot.

Starting with the model:

class IManipulatorModel(ABC):

  ...

This class will have functionality for the following (it loosely follows the existing utils functions in lite6_model_utils.py)
- Get the manipulator type
- Get num dof (just the main joints, so independent of EEF)
- Get the path of the description file (urdf, but I also want to maybe support other formats in the future, so let's keep it generic and call it description file)
- Get the names of some of the important frames (base, end effector, etc)
- Get the number of positions, states, etc. I think these are used for multibody state vector mapping. Please confirm that it is used for that and if it is useful, add these functions.
- there are methods for filling in the state vectors, etc in the current deprecated functions but I'm not sure if these are necessary. Also I want to switch to using the definition classes, so let's skip this for now.

^ All these above functions need to be abstract.
Also please don't port any "gripper" stuff from the deprecated stuff. I don't want gripperstatus enum etc for now. Also we're switching to calling it end-effector to support non gripper tools as well.

class Lite6Model(IManipulatorModel):
  model_type: IManipulatorModelType

We can crate dict mappings from the different model types to the functions required for the interface implementation (roughly following the existing deprecated utils)

Note that the goal is that we can use this model instance as the only thing required to work with Drake to set up sim and also do FK/IK.

class IManipulatorDriver(ABC):

  @abstractmethod
  def prime() -> None:
     // Prime the arm for use after boot up or after a reset.

  @abstractmethod
  def unprime() -> None:
    // Unprime the arm after use before shutdown or reset.

  @abstractmethod
  def read_joint_positions() -> JointPositions:
    ...

  @abstractmethod
  def read_eef_positions() -> EEFPositions | None:
    ...

  @abstractmethod
  def read_joint_velocities() -> JointVelocities:
    ...

  @abstractmethod
  def read_eef_velocities() -> EEFVelocities | None:
    ...

  @abstractmethod
  def write_joint_positions(joint_positions) -> None:
    ...

  @abstractmethod
  def write_eef_positions(eef_positions) -> None:
    ...

  @abstractmethod
  def write_joint_velocities(joint_velocities) -> None:
    ...

  @abstractmethod
  def write_eef_velocities(eef_velocities) -> None:
    ...


And then we have the Lite6Driver which implements the IManipulatorDriver interface and has the actual code for communicating with the robot (either through serial or through the network, etc)
Please figure out:
1. If the xarm API has methods to do these things (reading/writing joint/eef positions/velocities). I have some old code in the repo that you can use for reference, but it was also using a very old API version so you might wanna look up the latest API docs to see if there are any changes.
2. Figure out how (and if possible) to wrap the API calls into the above interface.

  


  
