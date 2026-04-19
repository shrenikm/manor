@0x9d41350d0ccfbc30;

using V1 = import "eef_state_trajectory_v1.capnp";

struct VersionedEefStateTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefStateTrajectoryV1;
    }
}
