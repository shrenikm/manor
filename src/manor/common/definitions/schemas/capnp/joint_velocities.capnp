@0x8aa4d7621345db02;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct JointVelocitiesV1 {
    header     @0 :Header.TimestampHeaderV1;
    velocities @1 :Common.Float64Array;
}

struct VersionedJointVelocities {
    union {
        unset @0 :Void;
        v1    @1 :JointVelocitiesV1;
    }
}
