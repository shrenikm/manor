@0x9249eeeb81572076;

using V1 = import "eef_twist_trajectory_v1.capnp";

struct VersionedEefTwistTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefTwistTrajectoryV1;
    }
}
