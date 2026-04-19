@0xe2ba6ddcc1e818e6;

using V1 = import "eef_positions_v1.capnp";

struct VersionedEefPositions {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefPositionsV1;
    }
}
