@0xf89e6fff3742cdc1;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFVelocitiesV1 {
    header     @0 :Header.TimestampHeaderV1;
    velocities @1 :Common.Float64Array;
}

struct VersionedEEFVelocities {
    union {
        unset @0 :Void;
        v1    @1 :EEFVelocitiesV1;
    }
}
