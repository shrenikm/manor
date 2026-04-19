@0xaad98fe4b2ed9326;

using V1 = import "joint_state_trajectory_v1.capnp";

struct VersionedJointStateTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.JointStateTrajectoryV1;
    }
}
