@0x8d92dad42f629a7f;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefPositionsTrajectoryV1 {
    header            @0 :Header.TimestampHeaderV1;
    times             @1 :Common.Float64Array;
    eefPositionsArray @2 :Common.Float64Array;
}
