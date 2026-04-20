@0xe2ba6ddcc1e818e6;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFPositionsV1 {
    header    @0 :Header.TimestampHeaderV1;
    positions @1 :Common.Float64Array;
}

struct VersionedEEFPositions {
    union {
        unset @0 :Void;
        v1    @1 :EEFPositionsV1;
    }
}
