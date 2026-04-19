@0xa608326ec9ddad27;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefVelocitiesV1 {
    header     @0 :Header.TimestampHeaderV1;
    velocities @1 :Common.Float64Array;
}
