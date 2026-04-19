@0xc72afb34f12a25f8;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct EefPositionsV1 {
    header    @0 :Header.TimestampHeaderV1;
    positions @1 :Common.Float64Array;
}
