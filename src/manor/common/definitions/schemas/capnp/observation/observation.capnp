@0x8b8af80462604d48;

using V1 = import "observation_v1.capnp";

struct VersionedObservation {
    union {
        unset @0 :Void;
        v1    @1 :V1.ObservationV1;
    }
}
