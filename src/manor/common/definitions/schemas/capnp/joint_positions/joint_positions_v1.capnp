@0x85f8e7f5d75b4d12;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct JointPositionsV1 {
    header    @0 :Header.TimestampHeaderV1;
    positions @1 :Common.Float64Array;
}
