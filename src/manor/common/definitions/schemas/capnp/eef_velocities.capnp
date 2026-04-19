@0xf89e6fff3742cdc1;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EefVelocitiesV1 {
    header     @0 :Header.TimestampHeaderV1;
    velocities @1 :Common.Float64Array;
}

struct VersionedEefVelocities {
    union {
        unset @0 :Void;
        v1    @1 :EefVelocitiesV1;
    }
}
