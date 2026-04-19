@0x8aa4d7621345db02;

using V1 = import "joint_velocities_v1.capnp";

struct VersionedJointVelocities {
    union {
        unset @0 :Void;
        v1    @1 :V1.JointVelocitiesV1;
    }
}
