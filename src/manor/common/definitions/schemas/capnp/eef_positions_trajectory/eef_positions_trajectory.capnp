@0xbd9a3b7de48b956a;

using V1 = import "eef_positions_trajectory_v1.capnp";

struct VersionedEefPositionsTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefPositionsTrajectoryV1;
    }
}
