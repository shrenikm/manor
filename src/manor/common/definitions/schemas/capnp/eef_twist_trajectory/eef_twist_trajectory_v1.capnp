@0xd9a76076a9c93da7;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefTwistTrajectoryV1 {
    header       @0 :Header.TimestampHeaderV1;
    times        @1 :Common.Float64Array;
    linearArray  @2 :Common.Float64Array;
    angularArray @3 :Common.Float64Array;
}
