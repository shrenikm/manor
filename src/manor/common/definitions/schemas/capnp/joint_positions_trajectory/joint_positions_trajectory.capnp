@0x93b1fd518508cf90;

using V1 = import "joint_positions_trajectory_v1.capnp";

struct VersionedJointPositionsTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.JointPositionsTrajectoryV1;
    }
}
