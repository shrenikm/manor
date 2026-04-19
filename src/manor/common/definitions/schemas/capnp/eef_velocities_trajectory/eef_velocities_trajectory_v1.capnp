@0x88f72c1742094ade;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefVelocitiesTrajectoryV1 {
    header             @0 :Header.TimestampHeaderV1;
    times              @1 :Common.Float64Array;
    eefVelocitiesArray @2 :Common.Float64Array;
}
