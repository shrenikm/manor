@0xd9cdde76041c9a7f;

using V1 = import "proprioception_v1.capnp";

struct VersionedProprioception {
    union {
        unset @0 :Void;
        v1    @1 :V1.ProprioceptionV1;
    }
}
