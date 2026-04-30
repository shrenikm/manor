## Plan for action/command definitions

I realize that there are some issues:

- The names EEFPose (for pose of the tip of the EEF) and EEFPositions for positions of the actual EEF are confusing
- Command actually contains the wrong thing - EEFPose/Twist instead of EEFPositions/Velocities
- We should call everything "EE" instead of "EEF"

So here's the plan, we segment these commands into 3 categories: Joint, Cartesian and EE where cartesian is the pose/velocity of the end-effector (task/operation space control)

So the main idea is that Metis outputs Actions (which can contain higher level things like cartesian trajectories) but Talos only takes in "commands" which only has joint level control (for both the main arm and the EE). So Kyber is tasked with converting the actions into commands which may require IK to go from cartesian to join and maybe some kind of tracking algorithm or trajectory optimization to go from trajectories to instantaneous commands in positions/velocities.

So basically the current EEFPose, EEFTwist will be renamed to CartesianPose and CartesianTwist, same with the trajectories.
Also EEFPositions and EEFVelocities will be renamed to EEPositions and EEVelocities.

Definitions:

All of these in different files.

// only one can be given
JointCommand:
  JointPositions
  JointVelocities

// only one can be given
JointTrajectoryCommand:
  JointPositionsTrajectory
  JointVelocitiesTrajectory

// only one can be given
CartesianCommand:
  CartesianPose
  CartesianVelocity

// only one can be given
CartesianTrajectoryCommand:
  CartesianPoseTrajectory
  CartesianTwistTrajectory

// only one can be given
EECommand:
  EEPositions
  EEVelocities

// only one can be given
EETrajectoryCommand:
  EEPositionsTrajectory
  EEVelocitiesTrajectory

The command class now becomes:

// Joint command must be given and EECommand is optional
Command:
  JointCommand
  EECommand

Action now becomes:

// Group1: Joint and cartesian commands. Exaclty 1 of these 4 must be set (others None/optional)
// Group2: EE commands. Both optional but a maximum of 1 can be set.
Action:
  JointCommand
  JointTrajectoryCommand
  CartesianCommand
  CartesianTrajectoryCommand
  EECommand
  EETrajectoryCommand


Please make sure:

- Follow the existing design patters while writing these
- Make sure each of these classes still follow the capnrpto and lcm stuff. Need to follow the same capnp versioning convention we have been using, etc.
- Please write thorough tests for everything. Make sure we have tests covering all combinations of the instances for things like commands and actions (where only 1 can be set etc. So we test that none being set raises an error, more than 1 raises and error and we verify everything (serialization, etc) for all combinations)
- Please also replace all "EEF" that refer to end effector to "EE". Also any "eef_var" to "ee_var" and so on. Update this in the entire code base, documentaation etc.
- Please also note any of these change sin existing documentaiton or other code to make updates.
