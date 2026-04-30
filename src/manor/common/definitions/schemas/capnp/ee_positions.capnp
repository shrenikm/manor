@0xe2ba6ddcc1e818e6;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEPositionsV1 {
    header    @0 :Header.VersionedTimestampHeader;
    positions @1 :Common.Float64Array;
}

struct VersionedEEPositions {
    union {
        unset @0 :Void;
        v1    @1 :EEPositionsV1;
    }
}
