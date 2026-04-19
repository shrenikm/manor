@0xf6ba0d1f08d199ed;

using V1 = import "eef_twist_v1.capnp";

struct VersionedEefTwist {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefTwistV1;
    }
}
