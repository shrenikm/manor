@0xd308f77bf52c226e;

using V1 = import "joint_state_v1.capnp";

struct VersionedJointState {
    union {
        unset @0 :Void;
        v1    @1 :V1.JointStateV1;
    }
}
