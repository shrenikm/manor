@0xf89e6fff3742cdc1;

using V1 = import "eef_velocities_v1.capnp";

struct VersionedEefVelocities {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefVelocitiesV1;
    }
}
