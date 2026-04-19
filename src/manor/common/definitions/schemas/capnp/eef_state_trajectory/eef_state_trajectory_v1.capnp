@0xc308f9990f5f1724;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefStateTrajectoryV1 {
    header             @0 :Header.TimestampHeaderV1;
    times              @1 :Common.Float64Array;
    eefPositionsArray  @2 :Common.Float64Array;
    eefVelocitiesArray @3 :Common.Float64Array;
}
