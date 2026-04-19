@0xdc2e0e3fdfb3847a;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefPoseTrajectoryV1 {
    header             @0 :Header.TimestampHeaderV1;
    times              @1 :Common.Float64Array;
    translationsArray  @2 :Common.Float64Array;
    orientationsArray  @3 :Common.Float64Array;
}
