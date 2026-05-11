@0xe111e26a4e054f79;

using Header = import "/timestamp_header.capnp";
using EP = import "/ee_positions.capnp";
using EV = import "/ee_velocities.capnp";

struct EEStateV1 {
    header        @0 :Header.VersionedTimestampHeader;
    eePositions  @1 :EP.VersionedEEPositions;
    eeVelocities @2 :EV.VersionedEEVelocities;
}

struct VersionedEEState {
    union {
        unset @0 :Void;
        v1    @1 :EEStateV1;
    }
}
