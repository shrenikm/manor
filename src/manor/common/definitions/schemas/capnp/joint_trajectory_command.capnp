@0xcd853c29eeb9c162;

using Header = import "/timestamp_header.capnp";
using JPT = import "/joint_positions_trajectory.capnp";
using JVT = import "/joint_velocities_trajectory.capnp";

struct JointTrajectoryCommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        jointPositionsTrajectory  @1 :JPT.VersionedJointPositionsTrajectory;
        jointVelocitiesTrajectory @2 :JVT.VersionedJointVelocitiesTrajectory;
    }
}

struct VersionedJointTrajectoryCommand {
    union {
        unset @0 :Void;
        v1    @1 :JointTrajectoryCommandV1;
    }
}
