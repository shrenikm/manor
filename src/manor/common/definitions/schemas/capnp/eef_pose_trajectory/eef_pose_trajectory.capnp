@0x8bcd32629790343e;

using V1 = import "eef_pose_trajectory_v1.capnp";

struct VersionedEefPoseTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefPoseTrajectoryV1;
    }
}
