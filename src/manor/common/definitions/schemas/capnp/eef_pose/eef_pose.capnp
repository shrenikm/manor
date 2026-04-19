@0x93842928d8f76085;

using V1 = import "eef_pose_v1.capnp";

struct VersionedEefPose {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefPoseV1;
    }
}
