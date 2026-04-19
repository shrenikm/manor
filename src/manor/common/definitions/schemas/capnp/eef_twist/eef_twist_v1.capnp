@0xd19d031a19bb0068;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefTwistV1 {
    header  @0 :Header.TimestampHeaderV1;
    linear  @1 :Common.Float64Array;
    angular @2 :Common.Float64Array;
}
