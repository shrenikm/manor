@0xa02e5147049cff0d;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct JointVelocitiesV1 {
    header     @0 :Header.TimestampHeaderV1;
    velocities @1 :Common.Float64Array;
}
