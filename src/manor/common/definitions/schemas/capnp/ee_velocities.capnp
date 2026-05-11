@0xf89e6fff3742cdc1;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEVelocitiesV1 {
    header     @0 :Header.VersionedTimestampHeader;
    velocities @1 :Common.Float64Array;
}

struct VersionedEEVelocities {
    union {
        unset @0 :Void;
        v1    @1 :EEVelocitiesV1;
    }
}
