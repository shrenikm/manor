@0xc9e1e9ad93ec68aa;

using V1 = import "joint_positions_v1.capnp";

struct VersionedJointPositions {
    union {
        unset @0 :Void;
        v1    @1 :V1.JointPositionsV1;
    }
}
