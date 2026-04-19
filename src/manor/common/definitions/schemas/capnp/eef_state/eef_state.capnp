@0xe111e26a4e054f79;

using V1 = import "eef_state_v1.capnp";

struct VersionedEefState {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefStateV1;
    }
}
