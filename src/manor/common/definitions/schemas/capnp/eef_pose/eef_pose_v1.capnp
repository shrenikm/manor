@0x9c1e76761040c156;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefPoseV1 {
    header      @0 :Header.TimestampHeaderV1;
    translation @1 :Common.Float64Array;
    orientation @2 :Common.Float64Array;
}
