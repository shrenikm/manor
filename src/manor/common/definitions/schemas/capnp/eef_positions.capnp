@0xe2ba6ddcc1e818e6;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EefPositionsV1 {
    header    @0 :Header.TimestampHeaderV1;
    positions @1 :Common.Float64Array;
}

struct VersionedEefPositions {
    union {
        unset @0 :Void;
        v1    @1 :EefPositionsV1;
    }
}
